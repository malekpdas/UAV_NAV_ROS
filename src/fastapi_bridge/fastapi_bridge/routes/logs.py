import os
import asyncio
from fastapi import APIRouter, Request, HTTPException
from fastapi.responses import StreamingResponse
from typing import AsyncGenerator

router = APIRouter(prefix="/logs", tags=["logs"])


async def log_generator(request: Request) -> AsyncGenerator[str, None]:
    log_file_path = request.app.state.log_file_path

    if not os.path.exists(log_file_path):
        yield f"Log file not found at: {log_file_path}\n"
        return

    try:
        with open(log_file_path, "r") as f:
            # 1) Send existing content
            for line in f:
                if await request.is_disconnected():
                    return
                yield line

            # 2) Tail the file
            while True:
                # Stop if server shutting down
                if request.app.state.is_shutting_down:
                    return

                # Stop if client disconnected
                if await request.is_disconnected():
                    return

                line = f.readline()
                if line:
                    yield line
                else:
                    await asyncio.sleep(0.1)

    except asyncio.CancelledError:
        # REQUIRED: allow FastAPI/Uvicorn to cancel cleanly
        raise
    except Exception as e:
        yield f"Error reading log file: {str(e)}\n"


@router.get("/stream")
async def stream_logs(request: Request):
    try:
        response = StreamingResponse(
            log_generator(request),
            media_type="text/plain"
        )
        response.headers["Cache-Control"] = "no-cache"
        return response

    except Exception as e:
        raise HTTPException(
            status_code=500,
            detail=f"Error streaming logs: {str(e)}"
        )
