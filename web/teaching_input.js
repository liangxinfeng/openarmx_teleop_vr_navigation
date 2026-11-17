/* Pure edge classifier: never changes axes, Grip, trigger or reset keys. */
(function (root) {
  class TeachingInput {
    constructor() { this.reset(); }
    reset() {
      this.down = false; this.started = 0; this.pending = null;
      this.longSent = false; this.chord = false; this.wasEnabled = false;
      this.releaseRequired = false;
    }
    tick({enabled, tracked=true, right=false, left=false, now=performance.now()}) {
      const events = [];
      if (!tracked || !enabled) {
        // In ordinary VR, an unused teaching module must not swallow JR.
        // Retain ownership only for tracking loss or a teaching hold
        // already in progress, until the operator releases the button.
        this.releaseRequired = right && (this.releaseRequired || !tracked || (this.wasEnabled && this.down));
        this.down = right; this.pending = null; this.chord = false;
        this.wasEnabled = enabled; return {owns: this.releaseRequired && right, events};
      }
      if (!this.wasEnabled && right) this.releaseRequired = true;
      this.wasEnabled = true;
      if (this.releaseRequired) {
        if (!right) { this.releaseRequired = false; this.down = false; }
        return {owns:true, events};
      }
      if (right && left) {
        // The existing both-stick desktop-mode gesture keeps priority.
        this.chord = true; this.pending = null;
      }
      if (right && !this.down) {
        this.started = now; this.longSent = false;
        if (this.pending !== null && now - this.pending > 300) {
          events.push('menu'); this.pending = null;
        }
      }
      if (right && !this.chord && !this.longSent && now - this.started >= 1000) {
        this.longSent = true; this.pending = null; events.push('toggle_record');
      }
      if (!right && this.down) {
        if (!this.chord && !this.longSent) {
          if (this.pending !== null && now - this.pending <= 300) {
            events.push('capture'); this.pending = null;
          } else this.pending = now;
        }
        this.chord = false;
      }
      if (!right && this.pending !== null && now - this.pending > 300) {
        events.push('menu'); this.pending = null;
      }
      this.down = right;
      return {owns:true, events};
    }
  }
  root.TeachingInput = TeachingInput;
  if (typeof module !== 'undefined') module.exports = TeachingInput;
})(globalThis);
