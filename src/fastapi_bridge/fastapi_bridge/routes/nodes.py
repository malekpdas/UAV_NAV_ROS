from fastapi import APIRouter, HTTPException, Request
from fastapi_bridge.helpers.bundle_nodes import bundle_node_descriptions

router = APIRouter(prefix="/nodes", tags=["nodes"])

@router.get("/get_all_nodes")
def get_nodes(request: Request):
    """
    Returns the bundled node descriptions by calling the bundler function directly.
    """
    try:
        # Access the ROS2 node from the app state
        ws_root = request.app.state.ws_root
        
        # We call the function directly without writing to a file
        data = bundle_node_descriptions(ws_root)
        return data
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error bundling nodes: {str(e)}")
