(() => {
    const mapImageElement = document.getElementById("map-image");

    if (!mapImageElement || typeof getContainedImageRect !== "function") {
        console.warn("Map render stability patch could not initialize.");
        return;
    }

    let lastKnownMapAspect = null;

    function containedRectFromAspect(elementRect, imageAspect) {
        const elementAspect = elementRect.width / elementRect.height;

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

    getContainedImageRect = function stableContainedImageRect(imgElement) {
        const elementRect = imgElement.getBoundingClientRect();
        const naturalWidth = imgElement.naturalWidth;
        const naturalHeight = imgElement.naturalHeight;

        if (naturalWidth > 0 && naturalHeight > 0) {
            lastKnownMapAspect = naturalWidth / naturalHeight;
        }

        if (!lastKnownMapAspect || elementRect.width <= 0 || elementRect.height <= 0) {
            return elementRect;
        }

        return containedRectFromAspect(elementRect, lastKnownMapAspect);
    };

    function redrawMapItems() {
        if (!latestStatus) {
            return;
        }

        updateRobotMarker(latestStatus);
        updateMapOverlay(latestStatus);
    }

    mapImageElement.addEventListener("load", () => {
        if (mapImageElement.naturalWidth > 0 && mapImageElement.naturalHeight > 0) {
            lastKnownMapAspect = mapImageElement.naturalWidth / mapImageElement.naturalHeight;
        }

        requestAnimationFrame(redrawMapItems);
    });

    window.addEventListener("resize", () => {
        requestAnimationFrame(redrawMapItems);
    });

    if (mapImageElement.naturalWidth > 0 && mapImageElement.naturalHeight > 0) {
        lastKnownMapAspect = mapImageElement.naturalWidth / mapImageElement.naturalHeight;
    }

    console.log("Charlie dashboard map render stability patch loaded");
})();
