import numpy as np
class AgentWrapper:
    """Wraps the local DRL inference engine."""
    
    def __init__(self, config):
        self.config = config
        self.state_dim = config.get("state_dim")
        self.action_dim = config.get("action_dim", 2)
        self.n_rl_interval = config.get("n_rl_interval")
        self.n_ch_total = config.get("n_ch_total")
        
        print(f"[Agent] Initialized with config: {self.config}")
        
        # Memory for the current episode's trajectory
        self.trajectory = [] 

    def reset(self, initial_state):
        """Clears trajectory memory for a new episode."""
        self.trajectory = []
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
        """Dummy CPU inference function."""
        # TODO: Load PyTorch/ONNX model and run forward pass here
        # For now, return a dummy action based on the state to prove it works
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
