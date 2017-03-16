(function($) {
  
  $.PushBack = function( attr ) {
    attr = attr || {};

    this.histlen = attr.histlen || 5000;
    
    this.max_rps = attr.max_rps || 15;
    this.max_period = attr.max_period || 65535;
    this.steps_per_rev = attr.steps_per_rev || 800;
    this.clock_speed = attr.clock_speed || 16e6;
    this.update_rate = attr.update_rate || 75;

    // acceleration limit
    this.max_accel = attr.max_accel || 2;  // rev/s/update
    this.max_decel = attr.max_decel || 5;  // rev/s/update
    this.prev_result = null;

    this.f1_offset = attr.f1_offset || parseInt(localStorage.f1_offset) || 0;
    this.f2_offset = attr.f2_offset || parseInt(localStorage.f2_offset) || 0;

    // adjust saved values for the previous nsteps
    // because motor positions are zeroed on reload
    var nsteps1 = parseInt(localStorage.nsteps1);
    var nsteps2 = parseInt(localStorage.nsteps2);
    if (nsteps1 || nsteps2) {
      localStorage.m1_end = parseInt(localStorage.m1_end) - nsteps1;
      localStorage.m2_end = parseInt(localStorage.m2_end) - nsteps2;
      localStorage.gap_min = parseInt(localStorage.getItem('gap_min')) - nsteps1 - nsteps2;
      localStorage.gap_max = parseInt(localStorage.getItem('gap_max')) - nsteps1 - nsteps2;
    }

    this.m1_end = attr.m1_end || parseInt(localStorage.m1_end) || 20000;
    this.m2_end = attr.m2_end || parseInt(localStorage.m2_end) || 20000;
    // limits on difference between motor positions (pincer)
    this.gap_min = attr.gap_min || parseInt(localStorage.gap_min) || -500;
    this.gap_max = attr.gap_max || parseInt(localStorage.gap_max) || 500;

    this.ignore_limits = attr.ignore_limits || false;

    this.state_history = []; // [ {state_obj}, {state_obj}, ... ]

    // force blip filtering
    this.MAX_DF = 400000;
    this.MAX_BLIP_LEN = 4;
    this.last_f = {};  // {'f1':..., 'f2':...}
    this.blip_len = {};  // {'f1':..., 'f2':...}

    this.socket = new WebSocket( attr.wsurl || 'ws://localhost:9999');
    this.socket.onmessage = this._onmessage.bind(this);
  };
  $.PushBack.prototype._onmessage = function(e) {
    var latest = JSON.parse(e.data);

    // latest has raw values from arduino:
    // f1  - raw forces
    // f2
    // nsteps1 - raw positions
    // nsteps2
    // p1 - raw periods (speed)
    // p2
    // sys_state - internal
    //   m1en
    //   m2en
    //   m1fwd
    //   m2fwd
    //   timefail
    //
    // intermediate values:
    // f1norm  - normalized forces
    // f2norm
    // m1vel - motor velocities (rps)
    // m2vel
    // m1pos - motor position (steps, zero-center)
    // m2pos
    //
    // final values:
    // x - horizontal position 0-1
    // y - vertical position 0-1
    // fx - horzontal force
    // fy - vertical force

    if (latest.sys_state.timefail)
      console.log('timefail');

    localStorage.setItem('nsteps1', latest.nsteps1);
    localStorage.setItem('nsteps2', latest.nsteps2);

    // filter force blips
    var filtered = {};
    ['f1', 'f2'].forEach(function(f) {
      var blip = Math.abs(latest[f] - this.last_f[f]) > this.MAX_DF;
      if (blip) {
        this.blip_len[f]++;
        console.log('force blip', f, latest[f] - this.last_f[f],
                    'len', this.blip_len[f]);
        if (this.blip_len[f] >= this.MAX_BLIP_LEN) {
          console.log('blip exceeded length');
          blip = false;
        }
      }
      if (blip) {
        filtered[f] = this.last_f[f];
      } else {
        filtered[f] = latest[f];
        this.last_f[f] = latest[f];
        this.blip_len[f] = 0;
      }
    }, this);

    // add transformed parameters
    latest.f1norm = filtered.f1 - this.f1_offset;
    latest.f2norm = this.f2_offset - filtered.f2;
    latest.m1vel = this.period2rps(latest.p1) * (latest.sys_state.m1fwd ? -1 : 1) * (latest.sys_state.m1en ? 1 : 0);
    latest.m2vel = this.period2rps(latest.p2) * (latest.sys_state.m2fwd ? 1 : -1) * (latest.sys_state.m2en ? 1 : 0);

    // calculate the width of the working area.
    // motor end points represent a distance from the motor zero.
    // the zero points of each motor should be offset so that
    // at the minimum gap distance they are matched.
    var width = this.m1_end + this.m2_end - this.gap_min;
    latest.m1pos = this.m1_end - latest.nsteps1 - width/2;
    latest.m2pos = latest.nsteps2 - this.m2_end + width/2;

    var stepsPerMM = 200;
    var trapMaxWidth = this.gap_max - this.gap_min;
    var trapWidth = (latest.m2pos - latest.m1pos) / trapMaxWidth;
    if (trapWidth < 0) trapWidth = 0;
    if (trapWidth > 1) trapWidth = 1;
    var trapHeight = 1.0 - 0.66 * trapWidth * trapWidth - 0.33 * trapWidth;
    if (trapHeight < 0) trapHeight = 0;
    if (trapHeight > 1) trapHeight = 1;

    latest.x = (latest.m1pos + latest.m2pos) / 2 / width + 0.5;
    latest.y = trapHeight;
    latest.fx = (latest.f1norm + latest.f2norm) / 2;
    latest.fy = (latest.f1norm - latest.f2norm) / 2 * (trapHeight + 1) / (trapWidth + 2);

    this.state_history.push(latest);

    if (this.state_history.length == 1)
      this.onload();

    // get a reply
    var result;
    if (this.reply)
      result = this.reply(latest);
    if (!result)
      result = {'m1vel':0, 'm2vel':0};

    // result:
    // m1vel - m1 velocity rps
    // m2vel - m2 velocity rps

    // limit speed
    ['m1vel', 'm2vel'].forEach(function(m) {
      if (result[m] > this.max_rps) result[m] = this.max_rps;
      if (result[m] < -this.max_rps) result[m] = -this.max_rps;
    }, this);

    // check limits
    if(!this.ignore_limits) {
      var avg = (result.m1vel + result.m2vel)/2;
      var m1decel = this.decel_dist(result.m1vel);
      var m2decel = this.decel_dist(result.m2vel);
      if (result.m1vel > result.m2vel && latest.m1pos + m1decel - m2decel >= latest.m2pos) {
        //console.log('limit narrow');
        result.m1vel = avg;
        result.m2vel = avg;
      }
      if (result.m2vel > result.m1vel && latest.m2pos - latest.m1pos + m2decel - m1decel > this.gap_max - this.gap_min) {
        //console.log('limit wide');
        result.m1vel = avg;
        result.m2vel = avg;
      }
      if (result.m1vel < 0 && latest.nsteps1 + m1decel > this.m1_end) {
        //console.log('limit left');
        result.m1vel = 0;
        result.m2vel = 0;
      }
      if (result.m2vel > 0 && latest.nsteps2 + m2decel > this.m2_end) {
        //console.log('limit right');
        result.m1vel = 0;
        result.m2vel = 0;
      }
    }

    // limit acceleration
    if (this.prev_result) {
      ['m1vel', 'm2vel'].forEach(function(m) {
        var dv = result[m] - this.prev_result[m];
        if ((result[m] < -this.max_accel && this.prev_result[m] > this.max_accel) ||
            (result[m] > this.max_accel && this.prev_result[m] < -this.max_accel) ||
            (Math.abs(result[m]) < Math.abs(this.prev_result[m]))) {
          // decelerating
          if (dv > this.max_decel) dv = this.max_decel;
          if (dv < -this.max_decel) dv = -this.max_decel;
        } else {
          // accelerating
          if (dv > this.max_accel) dv = this.max_accel;
          if (dv < -this.max_accel) dv = -this.max_accel;
        }
        result[m] = this.prev_result[m] + dv;
      }, this);
    }
    this.prev_result = result;

    // convert coordinates
    var ret = {'sys_state':{}};
    ret.p1 = this.rps2period(Math.abs(result.m1vel));
    ret.p2 = this.rps2period(Math.abs(result.m2vel));
    ret.sys_state.m1en = ret.p1 < this.max_period;
    ret.sys_state.m2en = ret.p2 < this.max_period;
    ret.sys_state.m1fwd = result.m1vel < 0;
    ret.sys_state.m2fwd = result.m2vel > 0;

    this.socket.send(JSON.stringify(ret));

    while (this.state_history.length > this.histlen)
      this.state_history.splice(0, 1);
  };
  $.PushBack.prototype.onload = function() {
    // Fired after the first datum has come in from the server.
    console.log("loaded.");
  };
  $.PushBack.prototype.reply = function(state) {
    // Default: do nothing.
    // Clients can override this function
    return null;
  };
  $.PushBack.prototype.rps2period = function(rps) {
    var steps_per_sec = this.steps_per_rev * 2 * rps;
    var period = Math.floor(this.clock_speed / steps_per_sec);
    if (period > this.max_period) period = this.max_period;
    return period;
  };
  $.PushBack.prototype.period2rps = function(period) {
    var steps_per_sec = this.clock_speed / period / 2;
    var rps = steps_per_sec / this.steps_per_rev;
    return rps;
  }
  $.PushBack.prototype.decel_dist = function(vel) {
    var steps_per_update = Math.abs(vel) * this.steps_per_rev / this.update_rate;
    var decel = this.max_decel * this.steps_per_rev / this.update_rate;  // steps/update^2
    var decel_dist = steps_per_update * steps_per_update / decel / 2;
    decel_dist += steps_per_update * 2;  // allow delay of two updates?
    return decel_dist;
  };
  $.PushBack.prototype.get_history = function(key, N) {
    // Get the last N elements of history for key 'N'
    var st_idx = Math.max(0, this.state_history.length - N);
    return this.state_history
      .slice(st_idx)
      .map(function(x) { return x[key]; });
  };
  $.PushBack.prototype.set_zero_force = function() {
    // Use recent history (30 ticks) to adjust force offsets
    var f1 = this.get_history('f1', 30);
    var f2 = this.get_history('f2', 30);
    this.f1_offset = f1.reduce(function(x,y) { return x+y; }, 0) / f1.length;
    this.f2_offset = f2.reduce(function(x,y) { return x+y; }, 0) / f2.length;
    localStorage.setItem('f1_offset', this.f1_offset);
    localStorage.setItem('f2_offset', this.f2_offset);
  };
  $.PushBack.prototype.set_motor_end = function(m, offset) {
    if (typeof offset === 'undefined')
      var offset = 0;
    this[m + '_end'] = this.get_history('nsteps' + m[1], 1)[0] - offset;
    localStorage.setItem(m + '_end', this[m + '_end']);
  };
  $.PushBack.prototype.get_gap_dist = function() {
    return this.get_history('nsteps1', 1)[0] + this.get_history('nsteps2', 1)[0];
  };
  $.PushBack.prototype.set_gap_min = function(offset) {
    if (typeof offset === 'undefined')
      var offset = 0;
    this.gap_min = this.get_gap_dist() + offset;
    localStorage.setItem('gap_min', this.gap_min);
  };
  $.PushBack.prototype.set_gap_max = function(offset) {
    if (typeof offset === 'undefined')
      var offset = 0;
    this.gap_max = this.get_gap_dist() - offset;
    localStorage.setItem('gap_max', this.gap_max);
  };

})(window);
