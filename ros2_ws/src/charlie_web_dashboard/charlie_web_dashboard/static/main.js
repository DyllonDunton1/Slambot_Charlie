const keys = {};
let buttonLinear = 0.0;
let buttonAngular = 0.0;

console.log("Charlie dashboard main.js loaded");

const linearSlider = document.getElementById("linear-speed");
const angularSlider = document.getElementById("angular-speed");

const linearReadout = document.getElementById("linear-speed-readout");
const angularReadout = document.getElementById("angular-speed-readout");
const statusReadout = document.getElementById("status-readout");

const logStatus = document.getElementById("log-status");


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
    } catch (error) {
        statusReadout.textContent = "Status connection error";
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

updateSliderReadouts();
updateStatus();