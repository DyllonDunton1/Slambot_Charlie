let initialPoseClickArmed = false;

function setInitialPoseButtonState(active) {
    const button = document.getElementById("set-initial-pose-click");
    if (!button) {
        return;
    }

    if (active) {
        button.classList.add("active");
        button.textContent = "Click Map for Initial Pose";
    } else {
        button.classList.remove("active");
        button.textContent = "Set Initial Pose";
    }
}

function parseYawDegrees(defaultYawDeg = 0.0) {
    const raw = window.prompt(
        "Initial yaw in degrees. 0 means the robot points along +X on the map.",
        defaultYawDeg.toFixed(1),
    );

    if (raw === null) {
        return null;
    }

    const yawDeg = Number(raw);
    if (!Number.isFinite(yawDeg)) {
        setNavStatus("Initial pose: invalid yaw", "error");
        return null;
    }

    return yawDeg;
}

async function publishInitialPoseFromMapClick(event) {
    if (!initialPoseClickArmed) {
        return;
    }

    event.preventDefault();
    event.stopPropagation();
    event.stopImmediatePropagation();

    if (!latestStatus || !latestStatus.map || !latestStatus.map.received) {
        setNavStatus("Initial pose: map is not available yet", "error");
        initialPoseClickArmed = false;
        setInitialPoseButtonState(false);
        return;
    }

    const world = screenToWorld(event, latestStatus.map);
    if (!world) {
        return;
    }

    const defaultYawDeg = latestStatus.robot_pose && latestStatus.robot_pose.received
        ? Number(latestStatus.robot_pose.yaw_deg || 0.0)
        : 0.0;
    const yawDeg = parseYawDegrees(defaultYawDeg);

    if (yawDeg === null) {
        initialPoseClickArmed = false;
        setInitialPoseButtonState(false);
        setNavStatus("Initial pose: canceled");
        return;
    }

    const payload = {
        x: world.x,
        y: world.y,
        yaw: yawDeg * Math.PI / 180.0,
    };

    try {
        setNavStatus("Initial pose: publishing...");
        const response = await fetch("/api/nav/initial_pose", {
            method: "POST",
            headers: { "Content-Type": "application/json" },
            body: JSON.stringify(payload),
        });
        const data = await response.json();

        if (!response.ok || !data.ok) {
            throw new Error(data.message || `HTTP ${response.status}`);
        }

        setNavStatus(
            `Initial pose set: x=${world.x.toFixed(2)}, y=${world.y.toFixed(2)}, yaw=${yawDeg.toFixed(1)}°`,
            "active",
        );
        await updateStatus();
    } catch (error) {
        console.error("Failed to set initial pose", error);
        setNavStatus(`Initial pose failed | ${error.message}`, "error");
    } finally {
        initialPoseClickArmed = false;
        setInitialPoseButtonState(false);
    }
}

(function initializeInitialPoseControls() {
    const button = document.getElementById("set-initial-pose-click");

    if (!button || !mapContainer) {
        return;
    }

    button.addEventListener("click", () => {
        initialPoseClickArmed = !initialPoseClickArmed;
        setInitialPoseButtonState(initialPoseClickArmed);

        if (initialPoseClickArmed) {
            setNavStatus("Initial pose: click Charlie's current location on the map", "active");
        } else {
            setNavStatus("Initial pose: canceled");
        }
    });

    // Capture phase lets this consume the click before main.js treats it as a
    // waypoint click.
    mapContainer.addEventListener("click", publishInitialPoseFromMapClick, true);
})();

(function loadNavDashboardExtras() {
    if (document.getElementById("nav-dashboard-extras-script")) {
        return;
    }

    const script = document.createElement("script");
    script.id = "nav-dashboard-extras-script";
    script.src = "/static/nav_dashboard_extras.js?v=1";
    document.body.appendChild(script);
})();
