const navMessageBuffer = [];
const navMessageKeys = new Set();
const navMessageLimit = 12;
let latestComputedPathPoseCount = 0;

function ensureNavMessageBufferElement() {
    let element = document.getElementById("nav-message-buffer");
    if (element) {
        return element;
    }

    const navStatusElement = document.getElementById("nav-status");
    if (!navStatusElement || !navStatusElement.parentElement) {
        return null;
    }

    element = document.createElement("div");
    element.id = "nav-message-buffer";
    element.className = "nav-message-buffer";
    element.textContent = "Nav messages: none yet.";
    navStatusElement.insertAdjacentElement("afterend", element);
    return element;
}

function navMessageTimestamp() {
    return new Date().toLocaleTimeString([], {
        hour: "2-digit",
        minute: "2-digit",
        second: "2-digit",
    });
}

function classifyNavMessageMode(text, mode) {
    if (mode === "error" || /failed|error|rejected|aborted|timeout|unavailable|empty path|0 poses/i.test(text)) {
        return "error";
    }

    if (mode === "active" || /computed|added|set|active|sent|succeeded|published/i.test(text)) {
        return "active";
    }

    return "normal";
}

function addNavMessage(text, mode = "normal") {
    if (!text) {
        return;
    }

    const cleaned = String(text).trim();
    if (!cleaned) {
        return;
    }

    // Keep the buffer useful. The idle/default status is still shown in the
    // single-line Nav field, but it should not fill the persistent diagnostic log.
    if (cleaned === "Nav: No waypoint navigation action yet." || cleaned === "Nav: No waypoint navigation action yet") {
        return;
    }

    const key = `${classifyNavMessageMode(cleaned, mode)}|${cleaned}`;
    if (navMessageKeys.has(key)) {
        return;
    }

    navMessageKeys.add(key);
    navMessageBuffer.unshift({
        time: navMessageTimestamp(),
        text: cleaned,
        mode: classifyNavMessageMode(cleaned, mode),
    });

    while (navMessageBuffer.length > navMessageLimit) {
        const removed = navMessageBuffer.pop();
        navMessageKeys.delete(`${removed.mode}|${removed.text}`);
    }

    renderNavMessageBuffer();
}

function renderNavMessageBuffer() {
    const element = ensureNavMessageBufferElement();
    if (!element) {
        return;
    }

    if (navMessageBuffer.length === 0) {
        element.textContent = "Nav messages: none yet.";
        element.classList.remove("active", "error");
        return;
    }

    element.innerHTML = "";

    for (const item of navMessageBuffer) {
        const row = document.createElement("div");
        row.className = `nav-message-row ${item.mode}`;

        const timeSpan = document.createElement("span");
        timeSpan.className = "nav-message-time";
        timeSpan.textContent = item.time;

        const textSpan = document.createElement("span");
        textSpan.className = "nav-message-text";
        textSpan.textContent = item.text;

        row.appendChild(timeSpan);
        row.appendChild(textSpan);
        element.appendChild(row);
    }

    element.classList.remove("active", "error");
    if (navMessageBuffer.some((item) => item.mode === "error")) {
        element.classList.add("error");
    } else {
        element.classList.add("active");
    }
}

function getCurrentPathPoseCount() {
    const statusPath = latestStatus?.navigation?.path;
    if (statusPath?.received && Array.isArray(statusPath.poses)) {
        return statusPath.poses.length;
    }

    return latestComputedPathPoseCount;
}

function explainEmptyPath() {
    return "Compute Path returned an empty path (0 poses). Follow Path was blocked. This usually means the start or a waypoint is outside the free global costmap, inside inflated obstacle space, still in unknown space, or AMCL/map->base_link was not valid when planning ran.";
}

async function computePathWithValidation(event) {
    event.preventDefault();
    event.stopPropagation();
    event.stopImmediatePropagation();

    try {
        latestComputedPathPoseCount = 0;
        setNavStatus("Nav: computing path...");
        const response = await fetch("/api/nav/compute_path", { method: "POST" });
        const data = await response.json();

        if (!response.ok || !data.ok) {
            throw new Error(data.message || `HTTP ${response.status}`);
        }

        latestComputedPathPoseCount = Number(data.path_pose_count || 0);

        if (latestComputedPathPoseCount < 2) {
            const message = explainEmptyPath();
            setNavStatus(`Nav: ${message}`, "error");
            await updateStatus();
            addNavMessage(message, "error");
            return;
        }

        setNavStatus(`Nav: path computed | poses ${latestComputedPathPoseCount}`, "active");
        await updateStatus();
    } catch (error) {
        console.error("Failed to compute path", error);
        setNavStatus(`Nav: compute failed | ${error.message}`, "error");
    }
}

async function followPathWithValidation(event) {
    event.preventDefault();
    event.stopPropagation();
    event.stopImmediatePropagation();

    const pathPoseCount = getCurrentPathPoseCount();
    if (pathPoseCount < 2) {
        const message = "Follow Path blocked because there is no valid computed path yet. Click Compute Path and confirm the blue path appears on the map first.";
        setNavStatus(`Nav: ${message}`, "error");
        addNavMessage(message, "error");
        return;
    }

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

function installValidatedNavButtonHandlers() {
    const computeButton = document.getElementById("compute-path");
    const followButton = document.getElementById("follow-path");
    const clearButton = document.getElementById("clear-waypoints");

    if (computeButton) {
        computeButton.addEventListener("click", computePathWithValidation, true);
    }

    if (followButton) {
        followButton.addEventListener("click", followPathWithValidation, true);
    }

    if (clearButton) {
        clearButton.addEventListener("click", () => {
            latestComputedPathPoseCount = 0;
            addNavMessage("Clearing waypoints and computed path.");
        }, true);
    }
}

// Wrap the existing status setter so short-lived button/action responses are
// preserved even when /api/status polling updates the main Nav line 250 ms later.
const originalSetNavStatus = setNavStatus;
setNavStatus = function persistentSetNavStatus(text, mode = "normal") {
    originalSetNavStatus(text, mode);
    addNavMessage(text, mode);
};

// Replace the overlay renderer with a quieter Nav2 visualization:
//   - small blue dots for waypoints
//   - blue line for computed path
// The red robot arrow remains handled separately by updateRobotMarker().
updateMapOverlay = function updateMapOverlayBlue(status) {
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
            latestComputedPathPoseCount = points.length;
            const polyline = document.createElementNS("http://www.w3.org/2000/svg", "polyline");
            polyline.setAttribute("points", points.join(" "));
            polyline.setAttribute("class", "planned-path-line planned-path-line-blue");
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
        circle.setAttribute("r", "4");
        circle.setAttribute("class", "waypoint-dot waypoint-dot-blue");
        mapOverlay.appendChild(circle);

        const label = document.createElementNS("http://www.w3.org/2000/svg", "text");
        label.setAttribute("x", screen.x + 7);
        label.setAttribute("y", screen.y - 7);
        label.setAttribute("class", "waypoint-label waypoint-label-blue");
        label.textContent = `${index + 1}`;
        mapOverlay.appendChild(label);
    });
};

installValidatedNavButtonHandlers();
renderNavMessageBuffer();
