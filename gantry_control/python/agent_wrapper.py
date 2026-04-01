import numpy as np

try:
    from agents.hardware_handoff_v2.path7_object_adapter import build_policy
except Exception:
    build_policy = None

class AgentWrapper:
    """Wraps the local DRL inference engine."""
    
    def __init__(self, config):
        self.config = config
        self.state_dim = config.get("state_dim")
        self.action_dim = config.get("action_dim", 2)
        self.n_rl_interval = config.get("n_rl_interval")
        self.n_ch_total = config.get("n_ch_total")
        self.policy = None
        self.use_object_policy = False
        
        print(f"[Agent] Initialized with config: {self.config}")

        self._init_policy()
        
        # Memory for the current episode's trajectory
        self.trajectory = [] 

    def _init_policy(self):
        """Initialize deployed policy adapter when available."""
        if build_policy is None:
            print("[Agent] Deployed object policy unavailable; using built-in fallback.")
            return

        try:
            package_dir = self.config.get(
                "policy_package_dir",
                "agents/hardware_handoff_v2",
            )
            device = self.config.get("policy_device", "cpu")
            signal_shape = tuple(self.config.get("signal_shape", (3, 3, 2)))
            self.policy = build_policy(
                package_dir=package_dir,
                device=device,
                signal_shape=signal_shape,
            )
            self.use_object_policy = True
            print(f"[Agent] Loaded deployed object policy from {package_dir} (device={device}).")
        except Exception as ex:
            print(f"[Agent] Failed to load deployed object policy: {ex}")
            print("[Agent] Falling back to built-in dummy controller.")
            self.policy = None
            self.use_object_policy = False

    def reset(self, initial_state):
        """Clears trajectory memory for a new episode."""
        self.trajectory = []
        if self.use_object_policy:
            self.policy.reset()
        # In a real DRL, this might reset LSTM hidden states
        return self._compute_action(initial_state)

    def step(self, state, reward, done, truncated, record=True):
        """Logs the transition and computes the next action."""
        # Store trajectory data for the HPC (if enabled)
        if record:
            self.trajectory.append({
                "state": state,
                "reward": reward,
                "done": done,
                "truncated": truncated
            })
        
        if done > 0.5:
            return None # Episode over, no action needed
            
        return self._compute_action(state)

    def _compute_action(self, state):
        """Compute control action from deployed policy or fallback agent."""
        if self.use_object_policy:
            obs = np.array(state, dtype=np.float32).reshape(self.n_rl_interval, self.n_ch_total)
            action = self.policy.act(
                observation=obs,
                reward=0.0,
                done=False,
                truncated=False,
                info={},
            )
            action_arr = np.asarray(action, dtype=np.float64).reshape(-1)
            return action_arr[: self.action_dim].tolist()

        # Fallback: dummy action based on state
        dummy_action = self.dumb_agent(state)
        # Ensure it matches expected dimension
        while len(dummy_action) < self.action_dim:
            dummy_action.append(0.0)
            
        return dummy_action[:self.action_dim]

    def get_trajectory(self):
        """Returns the rollout data collected this episode."""
        return self.trajectory

    def update_weights(self, new_weights):
        """Loads new neural network weights received from the HPC."""
        print(f"[Agent] Updating local weights... (Size: {len(new_weights)} bytes)")
        # TODO: Apply new weights to PyTorch/ONNX model

    def dumb_agent(self, state):
        """Dumb control"""
        state_arr = np.array(state).reshape(self.n_rl_interval,self.n_ch_total)

        t    = state_arr[:,0]
        xloc = state_arr[:,1]
        yloc = state_arr[:,2]
        xvel = state_arr[:,3]
        yvel = state_arr[:,4]

        vel = 0.2
        T = 5000
        ycent = 400
        a1 = 1.309 # 75°
        t1 = ycent/np.sin(a1)/vel

        if t[-1] < t1:
            angle = a1
            u_act = np.abs(np.cos(angle))*vel
            v_act = np.sin(angle)*vel
        else:
            angle = 2*np.pi*((t[-1] - t1)/T) + a1
            # u_act = vel*0.5 + np.abs(np.cos(angle))*vel*0.5
            # v_act = np.sin(angle)*vel
            u_act = np.abs(np.cos(angle))*vel
            v_act = np.sin(angle)*vel

        print(f'{t[-1]:12.1f},x={xloc[-1]:8.3f},y={yloc[-1]:8.3f},{angle:8.3f},{u_act:5.3f},{v_act:6.3f}')
        return [u_act.item(), v_act.item()]
