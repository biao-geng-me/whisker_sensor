class AgentWrapper:
    """Wraps the local DRL inference engine."""
    
    def __init__(self, config):
        self.config = config
        self.state_dim = config.get("state_dim", 4)
        self.action_dim = config.get("action_dim", 2)
        
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
        dummy_action = [state[0] * 0.1, state[1] * -0.1]
        
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