#!/usr/bin/env python3

from fastapi import FastAPI, Request
from fastapi.responses import HTMLResponse
from fastapi.templating import Jinja2Templates
import uvicorn

app = FastAPI()

templates = Jinja2Templates(directory="templates")

robot_state = {
    "status": "Idle",
    "fps": 0,
    "robot_x": 0,
    "robot_y": 0
}


@app.get("/", response_class=HTMLResponse)
async def home(request: Request):
    return templates.TemplateResponse(
        request=request,
        name="index.html",
        context={
            "state": robot_state
        }
    )


@app.post("/start", response_class=HTMLResponse)
async def start_tracking():
    robot_state["status"] = "Tracking"

    return f"""
    <div class="card">
        Status: {robot_state["status"]}
    </div>
    """


@app.post("/stop", response_class=HTMLResponse)
async def stop_tracking():
    robot_state["status"] = "Stopped"

    return f"""
    <div class="card">
        Status: {robot_state["status"]}
    </div>
    """


@app.get("/telemetry", response_class=HTMLResponse)
async def telemetry():
    robot_state["fps"] += 1
    robot_state["robot_x"] += 5
    robot_state["robot_y"] += 3

    return f"""
    <div class="card">
        <h3>Telemetry</h3>
        <p>FPS: {robot_state["fps"]}</p>
        <p>X: {robot_state["robot_x"]}</p>
        <p>Y: {robot_state["robot_y"]}</p>
    </div>
    """


if __name__ == "__main__":
    uvicorn.run(app, host="0.0.0.0", port=8000)