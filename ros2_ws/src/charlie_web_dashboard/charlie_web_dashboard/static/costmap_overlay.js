(() => {
    const globalCostmapToggle = document.getElementById("show-global-costmap");
    const globalCostmapImage = document.getElementById("global-costmap-image");
    const costmapStatus = document.getElementById("costmap-status");

    let showGlobalCostmap = false;
    let lastGoodCostmapLoadMs = 0;

    if (!globalCostmapToggle || !globalCostmapImage || !costmapStatus) {
        console.warn("Nav2 global costmap overlay elements are missing.");
        return;
    }

    function setCostmapStatus(text, mode = "normal") {
        costmapStatus.textContent = text;
        costmapStatus.classList.remove("active", "error");

        if (mode === "active") {
            costmapStatus.classList.add("active");
        }

        if (mode === "error") {
            costmapStatus.classList.add("error");
        }
    }

    function setOverlayVisible(visible) {
        globalCostmapImage.style.display = visible ? "block" : "none";
    }

    async function refreshGlobalCostmapStatus() {
        if (!showGlobalCostmap) {
            return;
        }

        try {
            const response = await fetch(`/api/costmap/global/status?t=${Date.now()}`);
            const status = await response.json();

            if (!response.ok) {
                throw new Error(`HTTP ${response.status}`);
            }

            if (!status.received) {
                setCostmapStatus("Global Costmap: waiting for /global_costmap/costmap");
                return;
            }

            const age = status.last_update_age_s ?? 0.0;
            const width = status.width ?? 0;
            const height = status.height ?? 0;
            const resolution = status.resolution ?? 0.0;
            const frameId = status.frame_id || "unknown";

            const mode = age > 2.0 ? "error" : "active";
            setCostmapStatus(
                `Global Costmap: ${width} x ${height} | ${resolution.toFixed(3)} m/cell | frame ${frameId} | age ${age.toFixed(1)} s`,
                mode
            );
        } catch (error) {
            console.error("Failed to read global costmap status", error);
            setCostmapStatus(`Global Costmap: status failed | ${error.message}`, "error");
        }
    }

    function refreshGlobalCostmapImage() {
        if (!showGlobalCostmap) {
            return;
        }

        globalCostmapImage.src = `/api/costmap/global/image?t=${Date.now()}`;
    }

    globalCostmapToggle.addEventListener("change", () => {
        showGlobalCostmap = globalCostmapToggle.checked;
        setOverlayVisible(showGlobalCostmap);

        if (showGlobalCostmap) {
            setCostmapStatus("Global Costmap: loading...");
            refreshGlobalCostmapImage();
            refreshGlobalCostmapStatus();
        } else {
            setCostmapStatus("Global Costmap: hidden");
            globalCostmapImage.removeAttribute("src");
        }
    });

    globalCostmapImage.addEventListener("load", () => {
        if (!showGlobalCostmap) {
            return;
        }

        lastGoodCostmapLoadMs = Date.now();
        setOverlayVisible(true);
        refreshGlobalCostmapStatus();
    });

    globalCostmapImage.addEventListener("error", () => {
        if (!showGlobalCostmap) {
            return;
        }

        const secondsSinceGoodLoad = (Date.now() - lastGoodCostmapLoadMs) / 1000.0;
        const hadPriorLoad = lastGoodCostmapLoadMs > 0;

        if (hadPriorLoad && secondsSinceGoodLoad < 3.0) {
            return;
        }

        setCostmapStatus("Global Costmap: waiting for /global_costmap/costmap", "error");
    });

    setInterval(refreshGlobalCostmapImage, 1000);
    setInterval(refreshGlobalCostmapStatus, 1000);

    setOverlayVisible(false);
    setCostmapStatus("Global Costmap: hidden");
})();
