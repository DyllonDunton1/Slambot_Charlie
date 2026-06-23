const keys = {};
let buttonLinear = 0.0;
let buttonAngular = 0.0;

let tuningFieldsPopulated = false;

console.log("Charlie dashboard main.js loaded");

const linearSlider = document.getElementById("linear-speed");
const angularSlider = document.getElementById("angular-speed");

const linearReadout = document.getElementById("linear-speed-readout");
const angularReadout = document.getElementById("angular-speed-readout");
const statusReadout = document.getElementById("status-readout");

const logStatus = document.getElementById("log-status");

const mapImage = document.getElementById("map-image");
const mapStatus = document.getElementById("map-status");
const downloadMapButton = document.getElementById("download-map");

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

applyTuningButton.addEventListener("click", applyTuning);
resetIntegralButton.addEventListener("click", resetIntegral);

saveCheckpointButton.addEventListener("click", saveCheckpoint);
loadLatestCheckpointButton.addEventListener("click", loadLatestCheckpoint);

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

function setCheckpointStatus(text, mode = "normal") {
    checkpointStatus.textContent = text;

    checkpointStatus.classList.remove("active");
    checkpointStatus.classList.remove("error");

    if (mode === "active") {
        checkpointStatus.classList.add("active");
    }

    if (mode === "error") {
        checkpointStatus.classList.add("error");
    }
}


async function saveCheckpoint() {
    try {
        setCheckpointStatus("Checkpoint: saving...");

        const response = await fetch("/api/checkpoint/save", {
            method: "POST",
        });

        const data = await response.json();

        if (!response.ok || !data.ok) {
            throw new Error(data.message || `HTTP ${response.status}`);
        }

        setCheckpointStatus(
            `Checkpoint: saved ${data.checkpoint}`,
            "active"
        );

        await updateStatus();
    } catch (error) {
        console.error("Failed to save checkpoint", error);
        setCheckpointStatus(`Checkpoint: save failed | ${error.message}`, "error");
    }
}


async function loadLatestCheckpoint() {
    try {
        setCheckpointStatus("Checkpoint: loading latest...");

        const response = await fetch("/api/checkpoint/load_latest", {
            method: "POST",
        });

        const data = await response.json();

        if (!response.ok || !data.ok) {
            throw new Error(data.message || `HTTP ${response.status}`);
        }

        setCheckpointStatus(
            `Checkpoint: loaded ${data.checkpoint}`,
            "active"
        );

        await updateStatus();
    } catch (error) {
        console.error("Failed to load checkpoint", error);
        setCheckpointStatus(`Checkpoint: load failed | ${error.message}`, "error");
    }
}

function setPoseStatus(text, mode = "normal") {
    poseStatus.textContent = text;

    poseStatus.classList.remove("active");
    poseStatus.classList.remove("error");

    if (mode === "active") {
        poseStatus.classList.add("active");
    }

    if (mode === "error") {
        poseStatus.classList.add("error");
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
        // Image is limited by width.
        displayedWidth = elementRect.width;
        displayedHeight = displayedWidth / imageAspect;
        offsetX = 0;
        offsetY = (elementRect.height - displayedHeight) / 2.0;
    } else {
        // Image is limited by height.
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

    const imageContentRect = getContainedImageRect(mapImage);
    const containerRect = mapImage.parentElement.getBoundingClientRect();

    const mapWidthCells = map.width;
    const mapHeightCells = map.height;
    const resolution = map.resolution;
    const originX = map.origin_x;
    const originY = map.origin_y;

    if (mapWidthCells <= 0 || mapHeightCells <= 0 || resolution <= 0) {
        robotMarker.style.display = "none";
        setPoseStatus("Pose: bad map metadata", "error");
        return;
    }

    const cellX = (pose.x - originX) / resolution;
    const cellY = (pose.y - originY) / resolution;

    // The map PNG is flipped vertically in the backend with np.flipud(image),
    // so convert map cell y to image y.
    const imagePixelX = cellX;
    const imagePixelY = mapHeightCells - cellY;

    const screenX = imageContentRect.left - containerRect.left
        + (imagePixelX / mapWidthCells) * imageContentRect.width;

    const screenY = imageContentRect.top - containerRect.top
        + (imagePixelY / mapHeightCells) * imageContentRect.height;

    robotMarker.style.display = "block";
    robotMarker.style.left = `${screenX}px`;
    robotMarker.style.top = `${screenY}px`;

    const yawDegForScreen = -pose.yaw_deg;

    robotMarker.style.transform =
        `translate(-50%, -50%) rotate(${yawDegForScreen}deg)`;

    const age = pose.last_update_age_s ?? 0.0;

    setPoseStatus(
        `Pose: x=${pose.x.toFixed(2)} m | y=${pose.y.toFixed(2)} m | yaw=${pose.yaw_deg.toFixed(1)}° | age ${age.toFixed(1)} s`,
        "active"
    );
}

function computeCommand() {
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

function setMapStatus(text, mode = "normal") {
    mapStatus.textContent = text;

    mapStatus.classList.remove("active");
    mapStatus.classList.remove("error");

    if (mode === "active") {
        mapStatus.classList.add("active");
    }

    if (mode === "error") {
        mapStatus.classList.add("error");
    }
}

function refreshMapImage() {
    if (!mapImage) {
        return;
    }

    mapImage.src = `/api/map/image?t=${Date.now()}`;
}

function setTuningStatus(text, mode = "normal") {
    tuningStatus.textContent = text;

    tuningStatus.classList.remove("active");
    tuningStatus.classList.remove("error");

    if (mode === "active") {
        tuningStatus.classList.add("active");
    }

    if (mode === "error") {
        tuningStatus.classList.add("error");
    }
}

async function sendTuningCommand(command) {
    try {
        const response = await fetch("/api/tuning", {
            method: "POST",
            headers: {
                "Content-Type": "application/json",
            },
            body: JSON.stringify(command),
        });

        if (!response.ok) {
            throw new Error(`HTTP ${response.status}`);
        }

        const data = await response.json();

        setTuningStatus(
            `Tuning: sent ${JSON.stringify(data.command)}`,
            "active"
        );

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
    await sendTuningCommand({
        reset_integral: true,
    });
}

async function sendCommand(command) {
    try {
        await fetch("/api/cmd_vel", {
            method: "POST",
            headers: {
                "Content-Type": "application/json",
            },
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
        await fetch("/api/stop", {
            method: "POST",
        });
    } catch (error) {
        console.error("Failed to stop robot", error);
    }
}

async function updateStatus() {
    try {
        const response = await fetch("/api/status");
        const status = await response.json();
        statusReadout.textContent = JSON.stringify(status, null, 2);

        if (status.checkpoint && status.checkpoint.message) {
            if (status.checkpoint.ok) {
                setCheckpointStatus(
                    `Checkpoint: ${status.checkpoint.message} | latest: ${status.checkpoint.latest_checkpoint}`,
                    "active"
                );
            } else {
                setCheckpointStatus(
                    `Checkpoint: ${status.checkpoint.message}`,
                    "normal"
                );
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
                setLogStatus(
                    `Log: stopped | samples: ${sampleCount} | duration: ${duration.toFixed(1)} s`
                );
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
        updateRobotMarker(status);

    } catch (error) {
        statusReadout.textContent = "Status connection error";
        setPoseStatus("Pose: status connection error", "error");
    }
}

function setLogStatus(text, mode = "normal") {
    logStatus.textContent = text;

    logStatus.classList.remove("active");
    logStatus.classList.remove("error");

    if (mode === "active") {
        logStatus.classList.add("active");
    }

    if (mode === "error") {
        logStatus.classList.add("error");
    }
}

async function startDebugLog() {
    console.log("Start log button pressed");
    try {
        const response = await fetch("/api/debug_log/start", {
            method: "POST",
        });

        if (!response.ok) {
            throw new Error(`HTTP ${response.status}`);
        }

        const data = await response.json();

        setLogStatus(
            `Log: recording | samples: ${data.sample_count} | duration: 0.0 s`,
            "active"
        );

        await updateStatus();
    } catch (error) {
        console.error("Failed to start debug log", error);
        setLogStatus("Log: failed to start", "error");
    }
}

async function stopDebugLog() {
    try {
        const response = await fetch("/api/debug_log/stop", {
            method: "POST",
        });

        if (!response.ok) {
            throw new Error(`HTTP ${response.status}`);
        }

        const data = await response.json();

        const duration = data.duration_s === null ? 0.0 : data.duration_s;

        setLogStatus(
            `Log: stopped | samples: ${data.sample_count} | duration: ${duration.toFixed(1)} s`
        );

        await updateStatus();
    } catch (error) {
        console.error("Failed to stop debug log", error);
        setLogStatus("Log: failed to stop", "error");
    }
}

async function clearDebugLog() {
    try {
        const response = await fetch("/api/debug_log/clear", {
            method: "POST",
        });

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
    const command = computeCommand();
    sendCommand(command);
}, 100);

setInterval(updateStatus, 250);
setInterval(refreshMapImage, 1000);

updateSliderReadouts();
updateStatus();
refreshMapImage();
