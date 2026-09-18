#!/usr/bin/python
#
# Copyright (C) 2026 by Roberto Calvo-Palomino
#
#
#  This programa is free software: you can redistribute it and/or modify
#  it under the terms of the GNU General Public License as published by
#  the Free Software Foundation, either version 3 of the License, or
#  (at your option) any later version.
#
#  Electrosense is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU General Public License for more details.
# 
#  You should have received a copy of the GNU General Public License
#  along with RTL-Spec.  If not, see <http://www.gnu.org/licenses/>.
# 
# 	Authors: Roberto Calvo-Palomino <roberto [dot] calvo [at] urjc [dot] es>
#


import mujoco
import mujoco.viewer
import time
import math
import argparse
import numpy as np
from collections import deque

def main():
    parser = argparse.ArgumentParser(description="MuJoCo Minicube Simulator")    
    parser.add_argument("--viewer", type=str, choices=["native", "viser"], default="native", help="Select the viewer (native or viser)")
    args = parser.parse_args()

    # Load the model
    model = mujoco.MjModel.from_xml_path("assets/minicube_repyz.xml")
    data = mujoco.MjData(model)

    print("Starting MuJoCo simulator...")

    # Create the figure for the plot    
    fig = mujoco.MjvFigure()
    history_len = 500
    history = deque(maxlen=history_len)
    
    if args.viewer == "native":
        mujoco.mjv_defaultFigure(fig)
        fig.flg_extend = 1
        fig.title = "Sinusoidal Control Wave"
        fig.xlabel = "Last steps"
        fig.linename[0] = "amplitude"
        fig.figurergba[3] = 0.5
        for i in range(history_len):
            fig.linedata[0][2 * i] = -float(i)
            fig.linedata[0][2 * i + 1] = 0.0
        fig.linepnt[0] = 0

    show_plot = False

    def key_callback(keycode):
        nonlocal show_plot
        if keycode == 80: # 'p'
            show_plot = not show_plot

    if args.viewer == "native":
        print("Press the 'p' key in the simulation window to show/hide the graph.")
    else:
        print("Viser web viewer activated. Open your web browser at the indicated address.")

    # Main loop
    def run_simulation_loop(viewer, get_show_plot):
        render_fps = 60
        time_per_render = 1.0 / render_fps
        last_render_time = time.time()
        actuator_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "servo_actuator")
        
        # Helper to check if viewer is running
        is_running = getattr(viewer, 'is_running', lambda: True)

        while is_running():
            step_start = time.time()
            
            
            if actuator_id != -1:
                amplitude = 1.0
                frequency = 1.0
                val = amplitude * math.sin(2 * math.pi * frequency * data.time)
                data.ctrl[actuator_id] = val
                history.append(float(val))
            
            mujoco.mj_step(model, data)
            
            current_time = time.time()
            if current_time - last_render_time >= time_per_render:
                plot_active = get_show_plot()
                
                if args.viewer == "native":
                    if plot_active:
                        n = len(history)
                        fig.linepnt[0] = n
                        if n > 0:
                            data_arr = np.fromiter(history, dtype=float, count=n)
                            if n >= 5:
                                lo = float(np.percentile(data_arr, 2.0))
                                hi = float(np.percentile(data_arr, 98.0))
                                span = max(hi - lo, 1e-6)
                                lo -= 0.25 * span
                                hi += 0.25 * span
                            else:
                                v = float(history[-1])
                                span = max(abs(v), 1e-3)
                                lo, hi = v - span, v + span
                            fig.range[1][0] = lo
                            fig.range[1][1] = hi
                            for i in range(n):
                                fig.linedata[0][2 * i + 1] = history[-1 - i]
                        
                        try:
                            vw = viewer.viewport.width
                            vh = viewer.viewport.height
                            vp_w = int(vw * 0.33)
                            vp_h = int(vh * 0.25)
                            vp = mujoco.MjrRect(max(0, vw - vp_w), max(0, vh - vp_h), vp_w, vp_h)
                        except Exception:
                            vp = mujoco.MjrRect(800, 50, 400, 200)
                        viewer.set_figures([(vp, fig)])
                    else:
                        viewer.set_figures([])
                    
                    viewer.sync()
                elif args.viewer == "viser":
                    viewer.update_from_mjdata(data)
                    
                last_render_time = current_time

            time_until_next_step = model.opt.timestep - (time.time() - step_start)
            if time_until_next_step > 0:
                time.sleep(time_until_next_step)

    # Init viewer
    if args.viewer == "native":
        with mujoco.viewer.launch_passive(model, data, key_callback=key_callback) as viewer:
            run_simulation_loop(viewer, lambda: show_plot)
    elif args.viewer == "viser":
        try:
            import viser
            from mjviser import ViserMujocoScene
        except ImportError:
            print("Error: The 'mjviser' or 'viser' library was not found.")            
            return
            
        server = viser.ViserServer()
        server.gui.configure_theme(dark_mode=True)
        scene = ViserMujocoScene(server, model, num_envs=1)
        scene.create_visualization_gui()

        print("\n=== VISER SERVER STARTED ===")
        print("Open this address in your browser: http://localhost:8080")
        print("===============================\n")

        run_simulation_loop(scene, lambda: False)

if __name__ == "__main__":
    main()
