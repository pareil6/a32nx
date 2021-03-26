// Lives here as a Util for now so it can be cleanly separate from FMCMainDisplay.
// Needs access to the FlightPlanManager

class NXFlightGuidance {
    constructor(mcdu) {
        this.mcdu = mcdu;
        this.lastLeg = null;
        this.currentLeg = null;
        this.nextLeg = null;
        this.lastWaypoint = null;
        this.fromWaypoint = null;
        this.toWaypoint = null;
        this.nextToWaypoint = null;
        this.crossTrackError = null;
        this.trackAngleError = null;
        this.turnCenter = null;
        this.turnRadius = null;
        this.turnActive = false;
    }

    update(_deltaTime) {
        const updatedWaypoints = this.updateWaypoints();
        if (updatedWaypoints) {
            this.updateLegs();

            // If the errors were low before, we should be tracking correctly,
            // so smooth the turn to the new track.
            if (Math.abs(this.crossTrackError) < 0.05 && Math.abs(this.trackAngleError) < 5) {
                this.turnCenter = null;
                this.turnRadius = null;
                this.turnActive = true;
            }

            //console.log(`WPs: (${this.lastWaypoint ? this.lastWaypoint.ident : "null"}) - ${this.fromWaypoint ? this.fromWaypoint.ident : null} - ${this.toWaypoint ? this.toWaypoint.ident : null} - (${this.nextToWaypoint ? this.nextToWaypoint.ident : null})`);
            //console.log(`Tracks: (${Math.round(lastLegTrack)}) - ${Math.round(activeLegTrack)} - (${Math.round(nextLegTrack)})`);
        }
        this.updateErrors();

        SimVar.SetSimVarValue("L:A32NX_FG_AVAIL", "bool", true);
        SimVar.SetSimVarValue("L:A32NX_FG_CROSS_TRACK_ERROR", "nautical miles", this.crossTrackError || 0);
        SimVar.SetSimVarValue("L:A32NX_FG_TRACK_ANGLE_ERROR", "degree", this.trackAngleError || 0);
        SimVar.SetSimVarValue("L:A32NX_FG_PHI_COMMAND", "degree", 0);
    }

    updateWaypoints() {
        const waypointCount = this.mcdu.flightPlanManager.getWaypointsCount();
        const activeIndex = this.mcdu.flightPlanManager.getActiveWaypointIndex();
        if (waypointCount < activeIndex + 1 || activeIndex < 1) {
            const wasSet = this.fromWaypoint !== null;
            this.lastWaypoint = null;
            this.fromWaypoint = null;
            this.toWaypoint = null;
            this.nextToWaypoint = null;
            return wasSet;
        }

        const lastWaypoint = activeIndex > 0 ? this.mcdu.flightPlanManager.getWaypoint(activeIndex - 2) || null : null;
        const fromWaypoint = this.mcdu.flightPlanManager.getWaypoint(activeIndex - 1) || null;
        const toWaypoint = this.mcdu.flightPlanManager.getWaypoint(activeIndex) || null;
        const nextToWaypoint = waypointCount >= activeIndex + 3 ? this.mcdu.flightPlanManager.getWaypoint(activeIndex + 1) || null : null;

        if (fromWaypoint && this.fromWaypoint !== null && fromWaypoint.ident === this.fromWaypoint.ident) {
            // nothing has changed
            return false;
        }

        this.lastWaypoint = lastWaypoint;
        this.fromWaypoint = fromWaypoint;
        this.toWaypoint = toWaypoint;
        this.nextToWaypoint = nextToWaypoint;

        return true;
    }

    updateLegs() {
        if (this.fromWaypoint === null || this.toWaypoint === null) {
            this.lastLeg = null;
            this.currentLeg = null;
            this.nextLeg = null;
            return;
        }

        const activeLegTrack = this._getTrackFromWaypoints(this.fromWaypoint, this.toWaypoint);

        let lastLegTrack = null;
        if (this.lastWaypoint) {
            lastLegTrack = this._getTrackFromWaypoints(this.lastWaypoint, this.fromWaypoint);
        }

        let nextLegTrack = null;
        if (this.nextToWaypoint) {
            nextLegTrack = this._getTrackFromWaypoints(this.toWaypoint, this.nextToWaypoint);
        }
    }

    _getTrackFromWaypoints(from, to) {
        return Avionics.Utils.computeGreatCircleHeading(
            from.infos.coordinates,
            to.infos.coordinates
        );
    }

    _getPointOnTrack(from, dist, track) {
        const earthRadius = 3440.1;
        const deg2rad = Math.PI / 180;

        const angDist = dist / earthRadius;
        const trackRad = deg2rad * track;

        const fromLat = deg2rad * from.lat;
        const fromLon = deg2rad * from.long;

        const lat = Math.asin(Math.sin(fromLat) * Math.cos(angDist) + Math.cos(fromLat) * Math.sin(angDist) * Math.cos(trackRad));
        const lon = fromLon + Math.atan2(Math.sin(trackRad) * Math.sin(angDist) * Math.cos(fromLat), Math.cos(angDist) - Math.sin(fromLat) * Math.sin(lat));

        return new LatLong(lat / deg2rad, lon / deg2rad);
    }

    updateErrors(_deltaTime) {
        if (this.fromWaypoint === null || this.toWaypoint === null) {
            this.crossTrackError = null;
            return;
        }

        const ppos = new LatLong(SimVar.GetSimVarValue("PLANE LATITUDE", "degree latitude"), SimVar.GetSimVarValue("PLANE LONGITUDE", "degree longitude"));

        // angle error
        const trueTrack = SimVar.GetSimVarValue("GPS GROUND TRUE TRACK", "degree");
        const desiredTrack = Avionics.Utils.computeGreatCircleHeading(
            this.fromWaypoint.infos.coordinates,
            this.toWaypoint.infos.coordinates,
        );
        const mod = (x, n) => x - Math.floor(x / n) * n;
        this.trackAngleError = mod((desiredTrack - trueTrack + 180), 360) - 180;

        // crosstrack error
        const bearingAC = Avionics.Utils.computeGreatCircleHeading(this.fromWaypoint.infos.coordinates, ppos);
        const bearingAB = Avionics.Utils.computeGreatCircleHeading(
            this.fromWaypoint.infos.coordinates,
            this.toWaypoint.infos.coordinates,
        );
        const distanceAC = Avionics.Utils.computeDistance(
            this.fromWaypoint.infos.coordinates,
            ppos,
        );
        const earthRadius = 3440.1;
        const deg2rad = Math.PI / 180;

        const desiredOffset = 0;
        const actualOffset = Math.asin(
            Math.sin(deg2rad * (distanceAC / earthRadius)) *
            Math.sin(deg2rad * (bearingAC - bearingAB))
        ) / deg2rad * earthRadius;
        this.crossTrackError = desiredOffset - actualOffset;

        // If we have an active turn and the cross-track error is significant,
        // use the turn center to calculate where we should be instead. Once
        // the error becomes suitably small, we can disable the turn again and
        // start using the leg directly (until the next leg).
        if (Math.abs(this.crossTrackError) >= 0.05 && Math.abs(this.trackAngleError) >= 5) {
            if (this.turnActive) {
                // If we don't have the center yet, calculate it. At the point
                // we start the turn, we want a circle arc from our current
                // location to the point equidistant on the other side of the
                // waypoint.
                if (this.turnCenter === null) {
                    const remDist = Avionics.Utils.computeGreatCircleDistance(ppos, this.fromWaypoint.infos.coordinates);
                    const centerTrack = mod(trueTrack + 450, 360);

                    // Radius is negative to the left, positive to the right,
                    // matching the trackAngleError (positive means turn to the
                    // right).
                    this.turnRadius = remDist / Math.tan(deg2rad * this.trackAngleError / 2);

                    // Fudge the centerpoint slightly closer, which has the
                    // effect of making us turn marginally too tight, to
                    // compensate for the AP lagging the guidance.
                    this.turnRadius = this.turnRadius * 0.98;

                    this.turnCenter = this._getPointOnTrack(ppos, this.turnRadius, centerTrack);
                }

                if (this.turnRadius > 0) { // Right turn
                    this.trackAngleError = mod(Avionics.Utils.computeGreatCircleHeading(ppos, this.turnCenter) - trueTrack + 270 + 180, 360) - 180;
                    this.crossTrackError = -this.turnRadius + Avionics.Utils.computeGreatCircleDistance(ppos, this.turnCenter);
                } else { // Left turn
                    this.trackAngleError = mod(Avionics.Utils.computeGreatCircleHeading(ppos, this.turnCenter) - trueTrack + 450 + 180, 360) - 180;
                    this.crossTrackError = -this.turnRadius - Avionics.Utils.computeGreatCircleDistance(ppos, this.turnCenter);
                }

            }
            console.log(ppos + " " + this.turnCenter + " " + this.turnRadius + " " + (this.turnCenter ? Avionics.Utils.computeGreatCircleDistance(ppos, this.turnCenter) : ""));
        } else {
            this.turnCenter = null;
            this.turnRadius = null;
            this.turnActive = false;
        }
    }
}
