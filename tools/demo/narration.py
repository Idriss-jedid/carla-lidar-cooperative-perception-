"""
Every spoken line of the editor walkthrough: (key, subtitle title, text).

The video is the app at work, live: no slides. The explanations (the paper, CARLA) are spoken while the editor is
used. tts.py turns each sentence into an MP3 (edge-tts); record.py says them by key while it drives the editor.
Lines must stay true for any run: numbers that change from run to run are shown on screen, not spoken.
"""

LINES = [
    # ------------------------------------------------------------------ the window
    ("intro", "The scenario editor",
     "This is our scenario editor, connected to CARLA. In this video I drive it live: I build the paper's scene, "
     "run it in CARLA, record the LiDARs, and look at the bird's-eye view of every connected car."),
    ("win1", "The window",
     "CARLA is already running in the background with Town 3, as the top right corner says."),
    ("win2", "The window",
     "At the top: the map, the background, buttons to fit the view, and undo and redo."),
    ("win3", "The window",
     "Below it, the simulation bar: Start, Pause, Stop and Record. A grey button cannot be used yet."),
    ("win4", "The window",
     "The big area is the map, drawn from the town's road description. The mouse wheel zooms and the right button "
     "pans."),
    ("win5", "The window",
     "The background can also be an aerial photo. It was taken inside CARLA, by a camera one kilometre up."),
    ("win6", "The window",
     "Back to the road view, which shows the lanes and their driving directions."),
    ("win7", "The window",
     "On the right, three tabs: Scene for the vehicles, Settings for the sensors and the traffic, and BEV for the "
     "results."),
    ("win8", "The window",
     "The menus hold the files and the CARLA tools: check the connection, draw the plan inside CARLA, or test "
     "that every vehicle fits."),

    # ------------------------------------------------------------------ placing the paper's vehicles
    ("paper1", "The paper's scene",
     "The paper studies cooperative perception: connected cars, called CAVs, share parts of their LiDAR scans, so "
     "that each one also sees what others see."),
    ("paper2", "The paper's scene",
     "Its Figure 2 shows four CAVs and eleven other cars at a junction of Town 3 with a sloped road. "
     "This is that junction. I place the cars where the figure shows them."),
    ("place1", "The paper's scene",
     "First the CAVs, the cars that carry a LiDAR. Key one selects them."),
    ("place2", "The paper's scene",
     "A dashed outline follows the mouse. Green means a car can go there. Red means it cannot, and the status "
     "bar at the bottom says why."),
    ("place3", "The paper's scene",
     "A click places the car on the centre of the nearest lane, facing the driving direction."),
    ("place4", "The paper's scene",
     "The four CAVs go where the figure shows them, around the junction."),
    ("place5", "The paper's scene",
     "Now the eleven other cars. Key two switches to traffic."),
    ("place6", "The paper's scene",
     "Each one snaps to its lane. Two cars can never overlap, and the yellow box is the paper's spawn area."),
    ("place7", "The paper's scene",
     "The table lists every car with its position, heading and lane."),

    # ------------------------------------------------------------------ editing tools
    ("edit1", "Editing",
     "Select a car to change it. The R key switches it between CAV and traffic. Twice brings it back."),
    ("edit2", "Editing",
     "Dragging slides it along its lane. Every change can be undone with Control Z."),
    ("edit3", "Editing",
     "A car can also have its own speed and a route. Press T, then click points on the roads: the editor traces "
     "the path along the lanes, through the junction."),
    ("edit4", "Editing",
     "In the paper the Traffic Manager drives every car, so I clear this route and keep the scene exactly as in "
     "the figure."),

    # ------------------------------------------------------------------ settings
    ("set1", "The paper's settings",
     "In Settings, the LiDAR already has the paper's values: 32 channels, 120 metres and one million points per "
     "second. Resting the mouse on any setting explains it."),
    ("set2", "The paper's settings",
     "The paper varies how often cars run a red light, from zero to one hundred percent, so I set that range."),
    ("set3", "The paper's settings",
     "It also uses at least twenty other cars. Nine random ones complete our eleven."),
    ("set4", "The paper's settings",
     "The recording will save one frame every ten ticks, two frames per simulated second, until I stop it."),

    # ------------------------------------------------------------------ start, and how CARLA works
    ("start1", "Start",
     "Now, Start. The editor writes the experiment file and asks CARLA to spawn the twenty-four cars."),
    ("carla1", "How CARLA works",
     "While it starts: CARLA is a server built on the Unreal Engine. It holds the town, the physics, the cars and "
     "the sensors. Our editor is a Python client that spawns the cars and reads the data."),
    ("start2", "Live",
     "Now the scene drives in real time. Each box is a car at its current position, with its real size. "
     "The CAVs show their speed."),
    ("start3", "Live",
     "The bar shows the simulated time and how fast it runs compared with real time. The dashed outlines are the "
     "starting positions I placed."),
    ("start4", "Live",
     "In the corner, CARLA's own 3D view, from a camera that follows CAV zero."),
    ("carla2", "How CARLA works",
     "CARLA runs in synchronous mode: the world only moves when we tick, five hundredths of a second at a time. "
     "CARLA's Traffic Manager drives the cars, like an autopilot."),

    # ------------------------------------------------------------------ pause
    ("pause1", "Pause",
     "Pause freezes everything. CARLA waits, the Traffic Manager waits, and nothing moves."),
    ("pause2", "Pause",
     "Resume continues exactly where it stopped."),

    # ------------------------------------------------------------------ record
    ("rec1", "Record",
     "Record mounts one LiDAR on each CAV, with the paper's settings, and starts saving."),
    ("rec2", "Record",
     "The four LiDARs take most of the computing time, so the simulation runs slower while recording."),
    ("carla3", "Record",
     "The LiDAR turns twenty times per second, so every tick gives one full 360 degree sweep, about twenty-seven "
     "thousand points. Each frame also stores every car's exact box from the same tick."),
    ("carla4", "Record",
     "That is the ground truth. It is used only to check the results, never to compute them."),
    ("rec4", "Record",
     "Stop recording. The LiDARs are removed, and the processing starts by itself: ego mask, road filter, ground "
     "removal, BEV and sectors, for every frame."),

    # ------------------------------------------------------------------ results
    ("bev1", "The BEV",
     "Here is the result. The BEV tab opened on the new recording. I pick its first frame, when the cars are "
     "still around the junction."),
    ("bev2", "The BEV",
     "Each coloured square is a half-metre cell where one CAV saw an obstacle, after the ground was removed. "
     "Each colour is one CAV."),
    ("bev3", "The BEV",
     "The grey outlines are the real car boxes from CARLA. Many squares fall on the side of a car that faces a "
     "CAV, and what lies behind it stays empty. The other squares are poles, walls and curbs at the road edge, or "
     "road surface that the ground removal left."),
    ("bev4", "The BEV",
     "The legend gives, for each CAV, its occupied cells and its obstacle points."),
    ("bev5", "The BEV",
     "Open figure shows one CAV step by step."),
    ("bev6", "The BEV",
     "First the raw sweep. Then only the drivable road: buildings and sidewalks are dropped. Then the ground "
     "removal keeps only the obstacles. Last, the BEV with its four sectors."),
    ("bev7", "The BEV",
     "Each sector has its size in megabytes and its quality, the share of obstacle surface it holds."),
    ("bev8", "The BEV",
     "In the paper, a central node uses these sector numbers to choose which parts to send to which car, under a "
     "bandwidth budget. That sharing step, and the fusion, come next."),

    # ------------------------------------------------------------------ every CAV, before and after filtering
    ("cmp1", "All CAVs, before and after filtering",
     "Now every CAV together, each figure on its own. Raw, all CAVs: the point clouds of the four CAVs superposed "
     "in the world frame, one colour per CAV. The grey rectangles are CARLA's boxes, drawn for every car."),
    ("cmp2", "All CAVs, before and after filtering",
     "After filtering: only the points left after the ego mask, the road filter and the ground removal. A box stays "
     "solid when a CAV has at least ten points on that car, and turns dashed when no CAV does."),
    ("cmp3", "All CAVs, before and after filtering",
     "The BEV of all CAVs together: every half-metre cell where at least one CAV saw an obstacle, darker where more "
     "points fall."),
    ("cmp4", "All CAVs, before and after filtering",
     "Points per car: for each car, the points on its body before and after filtering, and which CAVs see it. "
     "The title counts the cars that no LiDAR reached at all: too far away, or hidden behind other cars and "
     "buildings."),
    ("cmp5", "All CAVs, before and after filtering",
     "And each CAV on its own: its raw sweep, what is left after filtering, and its BEV. One CAV sees only some of "
     "the cars. The four together see more, which is what the paper's sharing is for."),

    # ------------------------------------------------------------------ stop
    ("stop1", "Stop",
     "Stop removes every car and puts CARLA back to normal."),
    ("stop2", "Stop",
     "For experiments that must be repeatable, the CARLA menu records a fixed number of frames in a separate "
     "console, the same way as the command line."),
    ("outro", "The scenario editor",
     "That is the whole chain, live: the paper's scene, CARLA, the LiDARs, and the BEV of every CAV."),
]

BY_KEY = {key: (title, text) for key, title, text in LINES}
