# Integration Guide

Complete code samples for integrating FastAPI Bridge into your frontend application.

## Complete Flight Control Workflow (JavaScript/React)

Full example of a pre-flight check, system launch, and flight operation.

```javascript
const API_BASE = "http://localhost:8000";

// Step 1: Discover available nodes
async function discoverNodes() {
  const res = await fetch(`${API_BASE}/nodes/get_all_nodes`);
  const nodes = await res.json();
  console.log("Found nodes:", nodes.map(n => `${n.executable_name} (${n.pkg_name})`));
  return nodes;
}

// Step 2: Read sensor configs
async function readSensorConfig(nodeId, exec, pkg) {
  const res = await fetch(`${API_BASE}/config/read`, {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify({ id: nodeId, exec, pkg, default: true })
  });
  return res.json();
}

// Step 3: Modify config if needed
async function updateSensorConfig(nodeId, updatedConfig) {
  const res = await fetch(`${API_BASE}/config/create`, {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify({ id: nodeId, config: updatedConfig })
  });
  return res.json();
}

// Step 4: Create launch file
async function createLaunchFile(nodesList) {
  const launchConfig = {
    launch: nodesList.map(n => ({
      pkg: n.pkg_name,
      exec: n.executable_name,
      id: n.node_default_id
    }))
  };
  
  const res = await fetch(`${API_BASE}/launch/create`, {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify(launchConfig)
  });
  return res.json();
}

// Step 5: Start the system
async function startSystem() {
  const res = await fetch(`${API_BASE}/launch/start`, { method: "POST" });
  const { pid, msg } = await res.json();
  console.log(`System started with PID ${pid}`);
  return pid;
}

// Step 6: Stream logs to UI
function streamLogs(logContainer) {
  const eventSource = new EventSource(`${API_BASE}/logs/stream`);
  
  eventSource.onmessage = (event) => {
    const line = event.data;
    logContainer.innerHTML += `${line}<br>`;
    logContainer.scrollTop = logContainer.scrollHeight; // Auto-scroll
  };
  
  eventSource.onerror = () => {
    console.error("Log stream error");
    eventSource.close();
  };
  
  return eventSource;
}

// Step 7: Start recording
async function startRecording(topics) {
  const res = await fetch(`${API_BASE}/launch/start_recording`, {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify({ topics })
  });
  const { bag_path } = await res.json();
  console.log(`Recording to: ${bag_path}`);
  return bag_path;
}

// Step 8: Stop the system
async function stopSystem() {
  const res = await fetch(`${API_BASE}/launch/stop`, { method: "POST" });
  console.log("System stopped");
  return res.json();
}

// Complete flight sequence
async function runFlight() {
  console.log("=== Pre-Flight Sequence ===");
  const nodes = await discoverNodes();
  
  // Get all nodes for full system
  const sensorNodes = nodes.filter(n => n.pub_topics.length > 0 && n.sub_topics.length === 0);
  const fusionNode = nodes.find(n => n.executable_name === "sensor_fusion");
  const controlNodes = nodes.filter(n => n.sub_topics.length > 0);
  
  const systemNodes = [...sensorNodes, fusionNode, ...controlNodes];
  
  // Verify configs
  for (const node of systemNodes) {
    const cfg = await readSensorConfig(node.node_default_id, node.executable_name, node.pkg_name);
    console.log(`${node.executable_name} config loaded:`, cfg.default ? "factory" : "custom");
  }
  
  // Create launch file
  await createLaunchFile(systemNodes);
  
  console.log("=== Starting Flight ===");
  const pid = await startSystem();
  
  // Monitor logs
  const logDiv = document.getElementById("system-logs");
  streamLogs(logDiv);
  
  // Wait for system to fully initialize (sensors, fusion ready)
  await new Promise(resolve => setTimeout(resolve, 5000));
  
  // Start recording
  const recordedTopics = [
    "fusion/odom",
    "imu_bno085/data",
    "gps_zoe_m8q/fix",
    "rc/channels"
  ];
  await startRecording(recordedTopics);
  
  console.log("=== READY FOR FLIGHT ===");
  
  // Flight duration...
  // After landing:
  await stopSystem();
  console.log("Flight complete, data recorded");
}
```

## React Component Example

Pre-flight checklist UI with real-time status.

```jsx
import { useState, useEffect } from 'react';

export function PreFlightChecklist() {
  const [nodes, setNodes] = useState([]);
  const [configs, setConfigs] = useState({});
  const [loading, setLoading] = useState(true);
  const [errors, setErrors] = useState([]);

  useEffect(() => {
    async function checkNodes() {
      try {
        const res = await fetch('http://localhost:8000/nodes/get_all_nodes');
        const nodeList = await res.json();
        setNodes(nodeList);
        
        // Check each node's config
        const cfgs = {};
        for (const node of nodeList) {
          const cfgRes = await fetch('http://localhost:8000/config/read', {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({
              id: node.node_default_id,
              exec: node.executable_name,
              pkg: node.pkg_name,
              default: false
            })
          });
          cfgs[node.node_default_id] = await cfgRes.json();
        }
        setConfigs(cfgs);
      } catch (err) {
        setErrors([...errors, err.message]);
      } finally {
        setLoading(false);
      }
    }
    
    checkNodes();
  }, []);

  if (loading) return <div>Loading nodes...</div>;

  return (
    <div style={{ fontFamily: 'Arial', padding: '20px' }}>
      <h2>Pre-Flight Checklist</h2>
      
      {errors.length > 0 && (
        <div style={{ color: 'red', marginBottom: '10px' }}>
          <strong>Errors:</strong>
          <ul>
            {errors.map((e, i) => <li key={i}>{e}</li>)}
          </ul>
        </div>
      )}
      
      <div>
        <h3>Sensors ({nodes.filter(n => n.pub_topics.length > 0 && n.sub_topics.length === 0).length})</h3>
        {nodes
          .filter(n => n.pub_topics.length > 0 && n.sub_topics.length === 0)
          .map(node => (
            <div key={node.node_default_id} style={{ marginLeft: '20px', marginBottom: '10px' }}>
              <strong>{node.executable_name}</strong>
              <ul>
                <li>
                  Config: {configs[node.node_default_id]?.default ? '✓ Factory' : '✓ Custom'}
                </li>
                <li>
                  Publishing: {node.pub_topics.map(t => t.name).join(', ')}
                </li>
              </ul>
            </div>
          ))}
      </div>
      
      <div>
        <h3>Fusion</h3>
        {nodes
          .filter(n => n.executable_name === 'sensor_fusion')
          .map(node => (
            <div key={node.node_default_id} style={{ marginLeft: '20px' }}>
              <strong>{node.executable_name}</strong>
              <ul>
                <li>
                  Subscribes to: {node.sub_topics.map(t => t.name).join(', ')}
                </li>
                <li>
                  Publishes: {node.pub_topics.map(t => t.name).join(', ')}
                </li>
              </ul>
            </div>
          ))}
      </div>
      
      <button onClick={() => window.location.reload()}>
        Refresh Status
      </button>
    </div>
  );
}
```

## System Monitor with Live Logs

Real-time log streaming component.

```jsx
import { useEffect, useRef } from 'react';

export function SystemMonitor() {
  const logsRef = useRef(null);
  const [isConnected, setIsConnected] = useState(false);

  useEffect(() => {
    const eventSource = new EventSource('http://localhost:8000/logs/stream');
    
    eventSource.onopen = () => setIsConnected(true);
    
    eventSource.onmessage = (event) => {
      if (logsRef.current) {
        const logLine = document.createElement('div');
        logLine.textContent = event.data;
        logLine.style.fontFamily = 'monospace';
        logLine.style.fontSize = '12px';
        logLine.style.padding = '2px 0';
        
        // Color code by severity
        if (event.data.includes('ERROR') || event.data.includes('permission denied')) {
          logLine.style.color = '#ff0000';
        } else if (event.data.includes('READY') || event.data.includes('initialized')) {
          logLine.style.color = '#00ff00';
        } else if (event.data.includes('WARNING')) {
          logLine.style.color = '#ffaa00';
        }
        
        logsRef.current.appendChild(logLine);
        logsRef.current.scrollTop = logsRef.current.scrollHeight;
      }
    };
    
    eventSource.onerror = () => {
      setIsConnected(false);
      eventSource.close();
    };
    
    return () => eventSource.close();
  }, []);

  return (
    <div style={{ 
      height: '500px', 
      border: '1px solid #ccc',
      backgroundColor: '#1e1e1e',
      color: '#00ff00',
      fontFamily: 'monospace',
      padding: '10px',
      overflowY: 'auto'
    }}>
      <div style={{ marginBottom: '10px' }}>
        Status: {isConnected ? '🟢 Connected' : '🔴 Disconnected'}
      </div>
      <div ref={logsRef} style={{ fontSize: '11px' }} />
    </div>
  );
}
```

## Configuration Editor

UI for modifying node parameters.

```jsx
import { useState } from 'react';

export function ConfigEditor({ nodeId, nodeExec, nodePkg }) {
  const [config, setConfig] = useState(null);
  const [loading, setLoading] = useState(true);
  const [saving, setSaving] = useState(false);

  const loadConfig = async () => {
    const res = await fetch('http://localhost:8000/config/read', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({
        id: nodeId,
        exec: nodeExec,
        pkg: nodePkg,
        default: false
      })
    });
    const data = await res.json();
    setConfig(data.config);
    setLoading(false);
  };

  const saveConfig = async () => {
    setSaving(true);
    try {
      const res = await fetch('http://localhost:8000/config/create', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({
          id: nodeId,
          config
        })
      });
      const result = await res.json();
      alert(`Config saved to: ${result.created}`);
    } catch (err) {
      alert(`Error: ${err.message}`);
    } finally {
      setSaving(false);
    }
  };

  if (loading) return <div>Loading...</div>;

  return (
    <div style={{ padding: '20px' }}>
      <h3>Edit {nodeExec} Configuration</h3>
      
      <textarea
        value={JSON.stringify(config, null, 2)}
        onChange={(e) => {
          try {
            setConfig(JSON.parse(e.target.value));
          } catch {
            // Invalid JSON, allow user to continue editing
          }
        }}
        style={{
          width: '100%',
          height: '400px',
          fontFamily: 'monospace',
          fontSize: '12px'
        }}
      />
      
      <div style={{ marginTop: '10px' }}>
        <button 
          onClick={saveConfig} 
          disabled={saving}
          style={{
            padding: '10px 20px',
            backgroundColor: '#4CAF50',
            color: 'white',
            border: 'none',
            borderRadius: '4px',
            cursor: 'pointer'
          }}
        >
          {saving ? 'Saving...' : 'Save Configuration'}
        </button>
        
        <button 
          onClick={loadConfig}
          style={{
            marginLeft: '10px',
            padding: '10px 20px',
            backgroundColor: '#999',
            color: 'white',
            border: 'none',
            borderRadius: '4px',
            cursor: 'pointer'
          }}
        >
          Reset
        </button>
      </div>
    </div>
  );
}
```

## Auto-Starting System on Page Load

Minimal example that auto-starts the system when page loads.

```javascript
async function autoStartSystem() {
  try {
    // Discover nodes
    const nodesRes = await fetch('http://localhost:8000/nodes/get_all_nodes');
    const nodes = await nodesRes.json();
    
    // Create launch
    const launchRes = await fetch('http://localhost:8000/launch/create', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({
        launch: nodes.map(n => ({
          pkg: n.pkg_name,
          exec: n.executable_name,
          id: n.node_default_id
        }))
      })
    });
    
    // Start system
    const startRes = await fetch('http://localhost:8000/launch/start', {
      method: 'POST'
    });
    const { pid } = await startRes.json();
    
    console.log(`System started with PID: ${pid}`);
    
    // Stream logs
    const logDiv = document.getElementById('logs');
    const eventSource = new EventSource('http://localhost:8000/logs/stream');
    eventSource.onmessage = (e) => {
      logDiv.innerHTML += e.data + '<br>';
      logDiv.scrollTop = logDiv.scrollHeight;
    };
    
  } catch (err) {
    console.error('Auto-start error:', err);
  }
}

// Run on page load
window.addEventListener('load', autoStartSystem);
```

## Error Handling Best Practices

```javascript
async function robustConfigRead(nodeId, exec, pkg) {
  try {
    const res = await fetch('http://localhost:8000/config/read', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ id: nodeId, exec, pkg, default: false }),
      timeout: 5000  // 5 second timeout
    });
    
    if (!res.ok) {
      throw new Error(`HTTP ${res.status}: ${res.statusText}`);
    }
    
    const data = await res.json();
    
    if (!data.config) {
      throw new Error('No config returned from API');
    }
    
    return data;
    
  } catch (err) {
    console.error(`Failed to read config for ${nodeId}:`, err);
    
    // Fallback strategy
    console.log('Attempting factory default...');
    const defaultRes = await fetch('http://localhost:8000/config/read', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ id: nodeId, exec, pkg, default: true })
    });
    
    return await defaultRes.json();
  }
}
```

---

**Next:** [Appendix & Resources](appendix.md)