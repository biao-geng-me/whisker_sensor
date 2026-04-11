The task at hand is to do live training on the double-carriage towing tank. Front one follow a path, back one is set to follow the front one with agent action infered from sensor data and self status.

Most of the infrastructure has been finished. We have a Python agent server in gantry_control/python/main_server_loop.py. And policy networks in gantry_control/python/agents. The pretrained ones runs ok. Now we need to train the one in gantry_control/python/agents/rl_sac_v4_pathblind_hardware, a very detailed readme is available therein.

The thing is that the hardware control side (MATLAB) has to be the main program. So the Server config (ServerConfigWindow.m) gets the settings and start the server. MATLAB and the agent server has a config handshake.

For training, take the seletected paths from carriage one and run randomized training. Can be only 1. The setup is similar to PathAgentPre, just now the model has to be updated.

The training is done locally. There are some artifact code about HPC, those should be discarded.

During the training, the agent can send reset message. Then the hardware side should reset. Such communication is not there yet with the infer mode.

Use the python environment in C:\Users\bigeme\py_envs\rl
