const keys = {};
let buttonLinear = 0.0;
let buttonAngular = 0.0;
let tuningFieldsPopulated = false;
let latestStatus = null;
let controlMode = "manual";

console.log("Charlie dashboard main.js loaded");

const linearSlider = document.getElementById("linear-speed");
const angularSlider = document.getElementById("angular-speed");
const linearReadout = document.getElementById("linear-speed-readout");
const angularReadout = document.getElementById("angular-speed-readout");
const statusReadout = document.getElementById("status-readout");
const logStatus = document.getElementById("log-status");

const mapContainer = document.getElementById("map-container");
const mapImage = document.getElementById("map-image");
const mapOverlay = document.getElementById("map-overlay");
const mapStatus = document.getElementById("map-status");
const mapSaveStatus = document.getElementById("map-save-status");
const downloadMapButton = document.getElementById("download-map");
const saveNavMapButton = document.getElementById("save-nav-map");

const tuningKpInput = document.getElementById("tuning-kp");
const tuningKiInput = document.getElementById("tuning-ki");
const tuningWheelRadiusInput = document.getElementById("tuning-wheel-radius");
const tuningWheelSeparationInput = document.getElementById("tuning-wheel-separation");
const applyTuningButton = document.getElementById("apply-tuning");
const resetIntegralButton = document.getElementById("reset-integral");
const tuningStatus = document.getElementById("tuning-status");

const poseStatus = document.getElementById("pose-status");
const robotMarker = document.getElementById("robot-marker");

const saveCheckpointButton = document.getElementById("save-checkpoint");
const loadLatestCheckpointButton = document.getElementById("load-latest-checkpoint");
const checkpointStatus = document.getElementById("checkpoint-status");

const imuStatus = document.getElementById("imu-status");

const batteryPill = document.getElementById("battery-pill");
const batteryVoltage = document.getElementById("battery-voltage");
const batteryPercent = document.getElementById("battery-percent");

const modePill = document.getElementById("mode-pill");
const controlModeLabel = document.getElementById("control-mode-label");
const modeManualButton = document.getElementById("mode-manual");
const modeWaypointNavButton = document.getElementById("mode-waypoint-nav");
const manualControlSection = document.getElementById("manual-control-section");
const waypointControlSection = document.getElementById("waypoint-control-section");
const computePathButton = document.getElementById("compute-path");
const followPathButton = document.getElementById("follow-path");
const clearWaypointsButton = document.getElementById("clear-waypoints");
const cancelNavButton = document.getElementById("cancel-nav");
const navStatus = document.getElementById("nav-status");
const waypointList = document.getElementById("waypoint-list");

function linearSpeed() {
    return parseFloat(linearSlider.value);
}

function angularSpeed() {
    return parseFloat(angularSlider.value);
}

function updateSliderReadouts() {
    linearReadout.textContent = linearSpeed().toFixed(2);
    angularReadout.textContent = angularSpeed().toFixed(2);
}

linearSlider.addEventListener("input", updateSliderReadouts);
angularSlider.addEventListener("input", updateSliderReadouts);

downloadMapButton.addEventListener("click", downloadMapPng);
saveNavMapButton.addEventListener("click", saveNavMap);
applyTuningButton.addEventListener("click", applyTuning);
resetIntegralButton.addEventListener("click", resetIntegral);
saveCheckpointButton.addEventListener("click", saveCheckpoint);
loadLatestCheckpointButton.addEventListener("click", loadLatestCheckpoint);
modeManualButton.addEventListener("click", () => setControlMode("manual"));
modeWaypointNavButton.addEventListener("click", () => setControlMode("waypoint_nav"));
computePathButton.addEventListener("click", computePath);
followPathButton.addEventListener("click", followPath);
clearWaypointsButton.addEventListener("click", clearWaypoints);
cancelNavButton.addEventListener("click", cancelNavigation);
mapContainer.addEventListener("click", handleMapClick);

function bindClick(id, handler) {
    const element = document.getElementById(id);

    if (!element) {
        console.error(`Missing button with id: ${id}`);
        return;
    }

    element.addEventListener("click", handler);
    console.log(`Bound click handler for #${id}`);
}

bindClick("start-log", startDebugLog);
bindClick("stop-log", stopDebugLog);
bindClick("clear-log", clearDebugLog);
bindClick("download-log", downloadDebugLog);

function updateBatteryHeader(status) {
    if (!batteryPill || !batteryVoltage || !batteryPercent) {
        return;
    }

    batteryPill.classList.remove("active", "battery-low", "battery-critical");

    const battery = status.battery;

    if (!battery || !battery.received || battery.voltage === null || battery.voltage === undefined) {
        batteryVoltage.textContent = "--.-- V";
        batteryPercent.textContent = "--%";
        return;
    }

    const voltage = Number(battery.voltage);
    const percentage = battery.percentage === null || battery.percentage === undefined
        ? null
        : Number(battery.percentage);

    batteryVoltage.textContent = Number.isNaN(voltage) ? "--.-- V" : `${voltage.toFixed(2)} V`;
    batteryPercent.textContent = percentage === null || Number.isNaN(percentage)
        ? "--%"
        : `${Math.round(percentage)}%`;

    const age = battery.last_update_age_s ?? 999.0;

    if (age > 2.0) {
        batteryPill.classList.add("battery-critical");
        return;
    }

    if (voltage <= 10.2) {
        batteryPill.classList.add("battery-critical");
    } else if (voltage <= 10.5) {
        batteryPill.classList.add("battery-low");
    } else {
        batteryPill.classList.add("active");
    }
}

function updateControlModeFromStatus(status) {
    const mode = status.control?.mode || "manual";
    controlMode = mode;

    controlModeLabel.textContent = mode === "waypoint_nav" ? "Waypoint Nav" : "Manual";

    modePill.classList.remove("active", "nav-active");
    modeManualButton.classList.remove("active");
    modeWaypointNavButton.classList.remove("active");
    manualControlSection.classList.remove("active-section", "hidden-section");
    waypointControlSection.classList.remove("active-section", "hidden-section");

    if (mode === "waypoint_nav") {
        modePill.classList.add("nav-active");
        modeWaypointNavButton.classList.add("active");
        manualControlSection.classList.add("hidden-section");
        waypointControlSection.classList.add("active-section");
    } else {
        modePill.classList.add("active");
        modeManualButton.classList.add("active");
        manualControlSection.classList.add("active-section");
        waypointControlSection.classList.add("hidden-section");
    }
}

function setImuStatus(text, mode = "normal") {
    imuStatus.textContent = text;
    imuStatus.classList.remove("active", "error");

    if (mode === "active") {
        imuStatus.classList.add("active");
    }

    if (mode === "error") {
        imuStatus.classList.add("error");
    }
}

function setCheckpointStatus(text, mode = "normal") {
    checkpointStatus.textContent = text;
    checkpointStatus.classList.remove("active", "error");

    if (mode === "active") {
        checkpointStatus.classList.add("active");
    }

    if (mode === "error") {
        checkpointStatus.classList.add("error");
    }
}

function setPoseStatus(text, mode = "normal") {
    poseStatus.textContent = text;
    poseStatus.classList.remove("active", "error");

    if (mode === "active") {
        poseStatus.classList.add("active");
    }

    if (mode === "error") {
        poseStatus.classList.add("error");
    }
}

function setMapStatus(text, mode = "normal") {
    mapStatus.textContent = text;
    mapStatus.classList.remove("active", "error");

    if (mode === "active") {
        mapStatus.classList.add("active");
    }

    if (mode === "error") {
        mapStatus.classList.add("error");
    }
}

function setMapSaveStatus(text, mode = "normal") {
    mapSaveStatus.textContent = text;
    mapSaveStatus.classList.remove("active", "error");

    if (mode === "active") {
        mapSaveStatus.classList.add("active");
    }

    if (mode === "error") {
        mapSaveStatus.classList.add("error");
    }
}

function setTuningStatus(text, mode = "normal") {
    tuningStatus.textContent = text;
    tuningStatus.classList.remove("active", "error");

    if (mode === "active") {
        tuningStatus.classList.add("active");
    }

    if (mode === "error") {
        tuningStatus.classList.add("error");
    }
}

function setLogStatus(text, mode = "normal") {
    logStatus.textContent = text;
    logStatus.classList.remove("active", "error");

    if (mode === "active") {
        logStatus.classList.add("active");
    }

    if (mode === "error") {
        logStatus.classList.add("error");
    }
}

function setNavStatus(text, mode = "normal") {
    navStatus.textContent = text;
    navStatus.classList.remove("active", "error");

    if (mode === "active") {
        navStatus.classList.add("active");
    }

    if (mode === "error") {
        navStatus.classList.add("error");
    }
}

async function saveCheckpoint() {
    try {
        setCheckpointStatus("Checkpoint: saving...");
        const response = await fetch("/api/checkpoint/save", { method: "POST" });
        const data = await response.json();

        if (!response.ok || !data.ok) {
            throw new Error(data.message || `HTTP ${response.status}`);
        }

        setCheckpointStatus(`Checkpoint: saved ${data.checkpoint}`, "active");
        await updateStatus();
    } catch (error) {
        console.error("Failed to save checkpoint", error);
        setCheckpointStatus(`Checkpoint: save failed | ${error.message}`, "error");
    }
}

async function loadLatestCheckpoint() {
    try {
        setCheckpointStatus("Checkpoint: loading latest...");
        const response = await fetch("/api/checkpoint/load_latest", { method: "POST" });
        const data = await response.json();

        if (!response.ok || !data.ok) {
            throw new Error(data.message || `HTTP ${response.status}`);
        }

        setCheckpointStatus(`Checkpoint: loaded ${data.checkpoint}`, "active");
        await updateStatus();
    } catch (error) {
        console.error("Failed to load checkpoint", error);
        setCheckpointStatus(`Checkpoint: load failed | ${error.message}`, "error");
    }
}

function getContainedImageRect(imgElement) {
    const elementRect = imgElement.getBoundingClientRect();
    const naturalWidth = imgElement.naturalWidth;
    const naturalHeight = imgElement.naturalHeight;

    if (naturalWidth <= 0 || naturalHeight <= 0) {
        return elementRect;
    }

    const elementAspect = elementRect.width / elementRect.height;
    const imageAspect = naturalWidth / naturalHeight;

    let displayedWidth;
    let displayedHeight;
    let offsetX;
    let offsetY;

    if (imageAspect > elementAspect) {
        displayedWidth = elementRect.width;
        displayedHeight = displayedWidth / imageAspect;
        offsetX = 0;
        offsetY = (elementRect.height - displayedHeight) / 2.0;
    } else {
        displayedHeight = elementRect.height;
        displayedWidth = displayedHeight * imageAspect;
        offsetX = (elementRect.width - displayedWidth) / 2.0;
        offsetY = 0;
    }

    return {
        left: elementRect.left + offsetX,
        top: elementRect.top + offsetY,
        width: displayedWidth,
        height: displayedHeight,
    };
}

function worldToScreen(map, x, y) {
    const imageContentRect = getContainedImageRect(mapImage);
    const containerRect = mapImage.parentElement.getBoundingClientRect();

    const mapWidthCells = map.width;
    const mapHeightCells = map.height;
    const resolution = map.resolution;
    const originX = map.origin_x;
    const originY = map.origin_y;

    if (mapWidthCells <= 0 || mapHeightCells <= 0 || resolution <= 0) {
        return null;
    }

    const cellX = (x - originX) / resolution;
    const cellY = (y - originY) / resolution;
    const imagePixelX = cellX;
    const imagePixelY = mapHeightCells - cellY;

    return {
        x: imageContentRect.left - containerRect.left
            + (imagePixelX / mapWidthCells) * imageContentRect.width,
        y: imageContentRect.top - containerRect.top
            + (imagePixelY / mapHeightCells) * imageContentRect.height,
    };
}

function screenToWorld(event, map) {
    const imageContentRect = getContainedImageRect(mapImage);

    const relX = event.clientX - imageContentRect.left;
    const relY = event.clientY - imageContentRect.top;

    if (relX < 0 || relY < 0 || relX > imageContentRect.width || relY > imageContentRect.height) {
        return null;
    }

    const mapWidthCells = map.width;
    const mapHeightCells = map.height;
    const resolution = map.resolution;
    const originX = map.origin_x;
    const originY = map.origin_y;

    if (mapWidthCells <= 0 || mapHeightCells <= 0 || resolution <= 0) {
        return null;
    }

    const cellX = (relX / imageContentRect.width) * mapWidthCells;
    const imageCellY = (relY / imageContentRect.height) * mapHeightCells;
    const cellY = mapHeightCells - imageCellY;

    return {
        x: originX + cellX * resolution,
        y: originY + cellY * resolution,
    };
}

function updateRobotMarker(status) {
    if (!status.map || !status.robot_pose) {
        robotMarker.style.display = "none";
        setPoseStatus("Pose: waiting");
        return;
    }

    const map = status.map;
    const pose = status.robot_pose;

    if (!map.received || !pose.received) {
        robotMarker.style.display = "none";

        if (pose.error) {
            setPoseStatus(`Pose: unavailable | ${pose.error}`, "error");
        } else {
            setPoseStatus("Pose: waiting");
        }

        return;
    }

    const screen = worldToScreen(map, pose.x, pose.y);

    if (!screen) {
        robotMarker.style.display = "none";
        setPoseStatus("Pose: bad map metadata", "error");
        return;
    }

    robotMarker.style.display = "block";
    robotMarker.style.left = `${screen.x}px`;
    robotMarker.style.top = `${screen.y}px`;

    const yawDegForScreen = -pose.yaw_deg;
    robotMarker.style.transform = `translate(-50%, -50%) rotate(${yawDegForScreen}deg)`;

    const age = pose.last_update_age_s ?? 0.0;

    setPoseStatus(
        `Pose: x=${pose.x.toFixed(2)} m | y=${pose.y.toFixed(2)} m | yaw=${pose.yaw_deg.toFixed(1)}° | age ${age.toFixed(1)} s`,
        "active"
    );
}

function updateMapOverlay(status) {
    while (mapOverlay.firstChild) {
        mapOverlay.removeChild(mapOverlay.firstChild);
    }

    if (!status.map || !status.map.received || !status.navigation) {
        return;
    }

    const containerRect = mapContainer.getBoundingClientRect();
    mapOverlay.setAttribute("viewBox", `0 0 ${containerRect.width} ${containerRect.height}`);

    const path = status.navigation.path;
    const waypoints = status.navigation.waypoints || [];

    if (path && path.received && Array.isArray(path.poses) && path.poses.length >= 2) {
        const points = [];

        for (const pose of path.poses) {
            const screen = worldToScreen(status.map, pose.x, pose.y);
            if (screen) {
                points.push(`${screen.x},${screen.y}`);
            }
        }

        if (points.length >= 2) {
            const polyline = document.createElementNS("http://www.w3.org/2000/svg", "polyline");
            polyline.setAttribute("points", points.join(" "));
            polyline.setAttribute("class", "planned-path-line");
            mapOverlay.appendChild(polyline);
        }
    }

    waypoints.forEach((waypoint, index) => {
        const screen = worldToScreen(status.map, waypoint.x, waypoint.y);
        if (!screen) {
            return;
        }

        const circle = document.createElementNS("http://www.w3.org/2000/svg", "circle");
        circle.setAttribute("cx", screen.x);
        circle.setAttribute("cy", screen.y);
        circle.setAttribute("r", "6");
        circle.setAttribute("class", "waypoint-dot");
        mapOverlay.appendChild(circle);

        const label = document.createElementNS("http://www.w3.org/2000/svg", "text");
        label.setAttribute("x", screen.x + 9);
        label.setAttribute("y", screen.y - 9);
        label.setAttribute("class", "waypoint-label");
        label.textContent = `${index + 1}`;
        mapOverlay.appendChild(label);
    });
}

function updateWaypointPanel(status) {
    const navigation = status.navigation;

    if (!navigation) {
        setNavStatus("Nav: waiting");
        waypointList.textContent = "No navigation status yet.";
        return;
    }

    const nav = navigation.status || {};
    const waypoints = navigation.waypoints || [];
    const path = navigation.path || {};

    if (nav.state === "active") {
        const remaining = nav.distance_remaining === null || nav.distance_remaining === undefined
            ? "--"
            : `${Number(nav.distance_remaining).toFixed(2)} m`;
        const posesRemaining = nav.number_of_poses_remaining ?? "--";
        setNavStatus(`Nav: active | remaining ${remaining} | poses ${posesRemaining}`, "active");
    } else if (["error", "rejected", "aborted"].includes(nav.state)) {
        setNavStatus(`Nav: ${nav.message || nav.state}`, "error");
    } else {
        setNavStatus(`Nav: ${nav.message || nav.state || "idle"}`);
    }

    if (waypoints.length === 0) {
        waypointList.textContent = "No waypoints yet.";
        return;
    }

    const lines = waypoints.map((waypoint, index) => {
        return `${index + 1}. x=${Number(waypoint.x).toFixed(2)}, y=${Number(waypoint.y).toFixed(2)}`;
    });

    if (path.received) {
        lines.push(`Path: ${path.poses.length} poses`);
    } else {
        lines.push("Path: not computed yet");
    }

    waypointList.textContent = lines.join("\n");
}

function computeCommand() {
    if (controlMode !== "manual") {
        return { linear_x: 0.0, angular_z: 0.0 };
    }

    let linear = buttonLinear;
    let angular = buttonAngular;

    if (keys["w"] || keys["arrowup"]) {
        linear += linearSpeed();
    }

    if (keys["s"] || keys["arrowdown"]) {
        linear -= linearSpeed();
    }

    if (keys["a"] || keys["arrowleft"]) {
        angular += angularSpeed();
    }

    if (keys["d"] || keys["arrowright"]) {
        angular -= angularSpeed();
    }

    linear = Math.max(-linearSpeed(), Math.min(linearSpeed(), linear));
    angular = Math.max(-angularSpeed(), Math.min(angularSpeed(), angular));

    return { linear_x: linear, angular_z: angular };
}

function refreshMapImage() {
    if (!mapImage) {
        return;
    }

    mapImage.src = `/api/map/image?t=${Date.now()}`;
}

async function sendTuningCommand(command) {
    try {
        const response = await fetch("/api/tuning", {
            method: "POST",
            headers: { "Content-Type": "application/json" },
            body: JSON.stringify(command),
        });

        if (!response.ok) {
            throw new Error(`HTTP ${response.status}`);
        }

        const data = await response.json();
        setTuningStatus(`Tuning: sent ${JSON.stringify(data.command)}`, "active");
        await updateStatus();
    } catch (error) {
        console.error("Failed to send tuning command", error);
        setTuningStatus("Tuning: failed", "error");
    }
}

async function applyTuning() {
    const command = {
        kp: parseFloat(tuningKpInput.value),
        ki: parseFloat(tuningKiInput.value),
        wheel_radius_m: parseFloat(tuningWheelRadiusInput.value),
        wheel_separation_m: parseFloat(tuningWheelSeparationInput.value),
    };

    await sendTuningCommand(command);
}

async function resetIntegral() {
    await sendTuningCommand({ reset_integral: true });
}

async function sendCommand(command) {
    if (controlMode !== "manual") {
        return;
    }

    try {
        await fetch("/api/cmd_vel", {
            method: "POST",
            headers: { "Content-Type": "application/json" },
            body: JSON.stringify(command),
        });
    } catch (error) {
        console.error("Failed to send command", error);
    }
}

async function stopRobot() {
    buttonLinear = 0.0;
    buttonAngular = 0.0;

    for (const key in keys) {
        keys[key] = false;
    }

    try {
        await fetch("/api/stop", { method: "POST" });
        controlMode = "manual";
        await updateStatus();
    } catch (error) {
        console.error("Failed to stop robot", error);
    }
}

async function setControlMode(mode) {
    try {
        const response = await fetch("/api/control_mode", {
            method: "POST",
            headers: { "Content-Type": "application/json" },
            body: JSON.stringify({ mode }),
        });
        const data = await response.json();

        if (!response.ok || !data.ok) {
            throw new Error(data.message || `HTTP ${response.status}`);
        }

        controlMode = data.mode;
        await updateStatus();
    } catch (error) {
        console.error("Failed to set control mode", error);
        setNavStatus(`Mode switch failed | ${error.message}`, "error");
    }
}

async function handleMapClick(event) {
    if (controlMode !== "waypoint_nav") {
        return;
    }

    if (!latestStatus || !latestStatus.map || !latestStatus.map.received) {
        setNavStatus("Nav: cannot add waypoint before map is available", "error");
        return;
    }

    const world = screenToWorld(event, latestStatus.map);
    if (!world) {
        return;
    }

    try {
        const response = await fetch("/api/nav/waypoints", {
            method: "POST",
            headers: { "Content-Type": "application/json" },
            body: JSON.stringify({ x: world.x, y: world.y, yaw: 0.0 }),
        });
        const data = await response.json();

        if (!response.ok || !data.ok) {
            throw new Error(data.message || `HTTP ${response.status}`);
        }

        setNavStatus(data.message, "active");
        await updateStatus();
    } catch (error) {
        console.error("Failed to add waypoint", error);
        setNavStatus(`Waypoint add failed | ${error.message}`, "error");
    }
}

async function computePath() {
    try {
        setNavStatus("Nav: computing path...");
        const response = await fetch("/api/nav/compute_path", { method: "POST" });
        const data = await response.json();

        if (!response.ok || !data.ok) {
            throw new Error(data.message || `HTTP ${response.status}`);
        }

        setNavStatus(`Nav: path computed | poses ${data.path_pose_count}`, "active");
        await updateStatus();
    } catch (error) {
        console.error("Failed to compute path", error);
        setNavStatus(`Nav: compute failed | ${error.message}`, "error");
    }
}

async function followPath() {
    try {
        setNavStatus("Nav: starting waypoint navigation...");
        const response = await fetch("/api/nav/follow_path", { method: "POST" });
        const data = await response.json();

        if (!response.ok || !data.ok) {
            throw new Error(data.message || `HTTP ${response.status}`);
        }

        setNavStatus(data.message, "active");
        await updateStatus();
    } catch (error) {
        console.error("Failed to start waypoint navigation", error);
        setNavStatus(`Nav: start failed | ${error.message}`, "error");
    }
}

async function clearWaypoints() {
    try {
        const response = await fetch("/api/nav/waypoints/clear", { method: "POST" });
        const data = await response.json();

        if (!response.ok || !data.ok) {
            throw new Error(data.message || `HTTP ${response.status}`);
        }

        setNavStatus(data.message);
        await updateStatus();
    } catch (error) {
        console.error("Failed to clear waypoints", error);
        setNavStatus(`Nav: clear failed | ${error.message}`, "error");
    }
}

async function cancelNavigation() {
    try {
        const response = await fetch("/api/nav/cancel", { method: "POST" });
        const data = await response.json();

        if (!response.ok || !data.ok) {
            throw new Error(data.message || `HTTP ${response.status}`);
        }

        setNavStatus(data.message);
        await updateStatus();
    } catch (error) {
        console.error("Failed to cancel navigation", error);
        setNavStatus(`Nav: cancel failed | ${error.message}`, "error");
    }
}

async function saveNavMap() {
    try {
        setMapSaveStatus("Nav Map: saving...");
        const response = await fetch("/api/map/save_nav", { method: "POST" });
        const data = await response.json();

        if (!response.ok || !data.ok) {
            throw new Error(data.message || `HTTP ${response.status}`);
        }

        setMapSaveStatus(`Nav Map: saved ${data.yaml_filename}`, "active");
    } catch (error) {
        console.error("Failed to save Nav2 map", error);
        setMapSaveStatus(`Nav Map: save failed | ${error.message}`, "error");
    }
}

async function updateStatus() {
    try {
        const response = await fetch("/api/status");
        const status = await response.json();
        latestStatus = status;
        statusReadout.textContent = JSON.stringify(status, null, 2);

        updateControlModeFromStatus(status);
        updateBatteryHeader(status);

        if (status.imu) {
            if (status.imu.received) {
                const age = status.imu.last_update_age_s ?? 0.0;
                const gzRad = status.imu.angular_velocity_z_radps ?? 0.0;
                const gzDps = status.imu.angular_velocity_z_dps ?? 0.0;
                const frameId = status.imu.frame_id || "unknown";
                const mode = age > 0.5 ? "error" : "active";

                setImuStatus(
                    `IMU: ${gzRad.toFixed(5)} rad/s | ${gzDps.toFixed(2)} °/s | frame ${frameId} | age ${age.toFixed(2)} s`,
                    mode
                );
            } else {
                setImuStatus("IMU: waiting");
            }
        }

        if (status.checkpoint && status.checkpoint.message) {
            if (status.checkpoint.ok) {
                setCheckpointStatus(
                    `Checkpoint: ${status.checkpoint.message} | latest: ${status.checkpoint.latest_checkpoint}`,
                    "active"
                );
            } else {
                setCheckpointStatus(`Checkpoint: ${status.checkpoint.message}`);
            }
        }

        if (!tuningFieldsPopulated && status.debug && status.debug.data) {
            const debug = status.debug.data;
            let populatedAnyField = false;

            if (debug.kp !== undefined) {
                tuningKpInput.value = Number(debug.kp).toFixed(3);
                populatedAnyField = true;
            }

            if (debug.ki !== undefined) {
                tuningKiInput.value = Number(debug.ki).toFixed(3);
                populatedAnyField = true;
            }

            if (debug.wheel_radius_m !== undefined) {
                tuningWheelRadiusInput.value = Number(debug.wheel_radius_m).toFixed(4);
                populatedAnyField = true;
            }

            if (debug.wheel_separation_m !== undefined) {
                tuningWheelSeparationInput.value = Number(debug.wheel_separation_m).toFixed(4);
                populatedAnyField = true;
            }

            if (populatedAnyField) {
                tuningFieldsPopulated = true;
                setTuningStatus("Tuning: loaded current values", "active");
            }
        }

        if (status.debug_log) {
            const sampleCount = status.debug_log.sample_count ?? 0;
            const duration = status.debug_log.duration_s ?? 0.0;

            if (status.debug_log.enabled) {
                setLogStatus(
                    `Log: recording | samples: ${sampleCount} | duration: ${duration.toFixed(1)} s`,
                    "active"
                );
            } else if (sampleCount > 0) {
                setLogStatus(`Log: stopped | samples: ${sampleCount} | duration: ${duration.toFixed(1)} s`);
            } else {
                setLogStatus("Log: idle");
            }
        }

        if (status.map) {
            if (status.map.received) {
                const age = status.map.last_update_age_s ?? 0.0;
                const width = status.map.width ?? 0;
                const height = status.map.height ?? 0;
                const resolution = status.map.resolution ?? 0.0;

                setMapStatus(
                    `Map: received | ${width} x ${height} | ${resolution.toFixed(3)} m/cell | age ${age.toFixed(1)} s`,
                    "active"
                );
            } else {
                setMapStatus("Map: waiting");
            }
        }

        updateWaypointPanel(status);
        updateRobotMarker(status);
        updateMapOverlay(status);

    } catch (error) {
        statusReadout.textContent = "Status connection error";
        setPoseStatus("Pose: status connection error", "error");
    }
}

async function startDebugLog() {
    console.log("Start log button pressed");
    try {
        const response = await fetch("/api/debug_log/start", { method: "POST" });

        if (!response.ok) {
            throw new Error(`HTTP ${response.status}`);
        }

        const data = await response.json();
        setLogStatus(`Log: recording | samples: ${data.sample_count} | duration: 0.0 s`, "active");
        await updateStatus();
    } catch (error) {
        console.error("Failed to start debug log", error);
        setLogStatus("Log: failed to start", "error");
    }
}

async function stopDebugLog() {
    try {
        const response = await fetch("/api/debug_log/stop", { method: "POST" });

        if (!response.ok) {
            throw new Error(`HTTP ${response.status}`);
        }

        const data = await response.json();
        const duration = data.duration_s === null ? 0.0 : data.duration_s;
        setLogStatus(`Log: stopped | samples: ${data.sample_count} | duration: ${duration.toFixed(1)} s`);
        await updateStatus();
    } catch (error) {
        console.error("Failed to stop debug log", error);
        setLogStatus("Log: failed to stop", "error");
    }
}

async function clearDebugLog() {
    try {
        const response = await fetch("/api/debug_log/clear", { method: "POST" });

        if (!response.ok) {
            throw new Error(`HTTP ${response.status}`);
        }

        await response.json();
        setLogStatus("Log: cleared");
        await updateStatus();
    } catch (error) {
        console.error("Failed to clear debug log", error);
        setLogStatus("Log: failed to clear", "error");
    }
}

async function downloadDebugLog() {
    try {
        setLogStatus("Log: preparing download...");
        const response = await fetch("/api/debug_log/download");

        if (!response.ok) {
            throw new Error(`HTTP ${response.status}`);
        }

        const blob = await response.blob();
        let filename = "charlie_debug_log.csv";
        const disposition = response.headers.get("Content-Disposition");

        if (disposition) {
            const match = disposition.match(/filename="(.+)"/);
            if (match && match[1]) {
                filename = match[1];
            }
        }

        const url = window.URL.createObjectURL(blob);
        const link = document.createElement("a");
        link.href = url;
        link.download = filename;
        document.body.appendChild(link);
        link.click();
        document.body.removeChild(link);
        window.URL.revokeObjectURL(url);

        setLogStatus(`Log: downloaded ${filename}`);
    } catch (error) {
        console.error("Failed to download debug log", error);
        setLogStatus("Log: download failed", "error");
    }
}

async function downloadMapPng() {
    try {
        setMapStatus("Map: preparing download...");
        const response = await fetch("/api/map/download");

        if (!response.ok) {
            throw new Error(`HTTP ${response.status}`);
        }

        const blob = await response.blob();
        let filename = "charlie_map.png";
        const disposition = response.headers.get("Content-Disposition");

        if (disposition) {
            const match = disposition.match(/filename="(.+)"/);
            if (match && match[1]) {
                filename = match[1];
            }
        }

        const url = window.URL.createObjectURL(blob);
        const link = document.createElement("a");
        link.href = url;
        link.download = filename;
        document.body.appendChild(link);
        link.click();
        document.body.removeChild(link);
        window.URL.revokeObjectURL(url);

        setMapStatus(`Map: downloaded ${filename}`, "active");
    } catch (error) {
        console.error("Failed to download map PNG", error);
        setMapStatus("Map: download failed", "error");
    }
}

function bindHoldButton(id, linearValue, angularValue) {
    const button = document.getElementById(id);

    function press(event) {
        event.preventDefault();
        if (controlMode !== "manual") {
            return;
        }
        buttonLinear = linearValue();
        buttonAngular = angularValue();
    }

    function release(event) {
        event.preventDefault();
        buttonLinear = 0.0;
        buttonAngular = 0.0;
    }

    button.addEventListener("mousedown", press);
    button.addEventListener("mouseup", release);
    button.addEventListener("mouseleave", release);
    button.addEventListener("touchstart", press);
    button.addEventListener("touchend", release);
}

bindHoldButton("forward", () => linearSpeed(), () => 0.0);
bindHoldButton("backward", () => -linearSpeed(), () => 0.0);
bindHoldButton("left", () => 0.0, () => angularSpeed());
bindHoldButton("right", () => 0.0, () => -angularSpeed());

document.getElementById("stop").addEventListener("click", stopRobot);

document.addEventListener("keydown", (event) => {
    keys[event.key.toLowerCase()] = true;

    if (event.key === " ") {
        stopRobot();
    }
});

document.addEventListener("keyup", (event) => {
    keys[event.key.toLowerCase()] = false;
});

setInterval(() => {
    if (controlMode !== "manual") {
        return;
    }

    const command = computeCommand();
    sendCommand(command);
}, 100);

setInterval(updateStatus, 250);
setInterval(refreshMapImage, 1000);

updateSliderReadouts();
updateStatus();
refreshMapImage();
