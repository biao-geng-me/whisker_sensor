# rov_mesh.npz: where it came from

Blue Robotics BlueROV2 R4, from Blue Robotics' public CAD (https://cad.bluerobotics.com/BLUEROV2-R4.zip, `BLUEROV2-R4-PUBLIC.STEP`), licensed CC BY 4.0 (Blue Robotics). Tessellated by ~/Projects/rov-cad `render/tessellate_bluerov2.py`, then reduced to the frame, thrusters, enclosures and buoyancy and decimated by vertex clustering with `wavi_py/tools/export_rov_mesh.py` (re-run it to rebuild). A display silhouette, not CAD.

Frame: rov-cad collar frame: forward +z, up +y, port +x; mm (R4 minus [0.0, 0.65, 165.628] mm), so it shares the whisker dome mesh's frame. Bounding box [[-169.0, -187.10000610351562, -389.8999938964844], [169.0, 64.9000015258789, 65.5]] mm.

- thrusters: 84 solids, 669260 -> 8991 triangles (cell 9.5 mm), 2 products
- enclosures: 9 solids, 279940 -> 4716 triangles (cell 7.8 mm), 7 products
- buoyancy: 8 solids, 78088 -> 4994 triangles (cell 8.8 mm), 2 products
- frame: 13 solids, 104428 -> 8485 triangles (cell 7.7 mm), 7 products
