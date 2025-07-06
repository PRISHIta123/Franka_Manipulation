import torch
import torch.nn as nn
import pickle

# Load expert data
with open("trajectory_data.pkl", "rb") as f:
    data = pickle.load(f)

states = torch.tensor([s for s, _ in data], dtype=torch.float32)
targets = torch.tensor([a for _, a in data], dtype=torch.float32)

# Simple MLP policy
class TrajectoryPolicy(nn.Module):
    def __init__(self):
        super().__init__()
        self.net = nn.Sequential(
            nn.Linear(7, 128),
            nn.ReLU(),
            nn.Linear(128, 128),
            nn.ReLU(),
            nn.Linear(128, 7)
        )

    def forward(self, x):
        return self.net(x)

policy = TrajectoryPolicy()
optimizer = torch.optim.Adam(policy.parameters(), lr=1e-3)
loss_fn = nn.MSELoss()

# Training loop
for epoch in range(100):
    optimizer.zero_grad()
    output = policy(states)
    loss = loss_fn(output, targets)
    loss.backward()
    optimizer.step()
    print(f"Epoch {epoch+1}, Loss: {loss.item():.6f}")

torch.save(policy.state_dict(), "trajectory_policy.pth")
print("Model saved.")
