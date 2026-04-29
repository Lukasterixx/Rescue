import torch
from pathlib import Path

old_path = Path(
    "/home/lukas/Rescue/Isaac/go2_omniverse/logs/rsl_rl/unitree_go2_rough/2024-04-06_02-37-07/model_7850.pt"
)

new_path = old_path.with_name("model_7850_converted.pt")

old = torch.load(old_path, map_location="cpu")
old_sd = old["model_state_dict"]

actor_sd = {}
critic_sd = {}

for k, v in old_sd.items():
    if k.startswith("actor."):
        # old: actor.0.weight
        # new: mlp.0.weight
        new_k = "mlp." + k[len("actor."):]
        actor_sd[new_k] = v

    elif k.startswith("critic."):
        # old: critic.0.weight
        # new: mlp.0.weight
        new_k = "mlp." + k[len("critic."):]
        critic_sd[new_k] = v

# New rsl_rl wants these top-level keys
new = {
    "actor_state_dict": actor_sd,
    "critic_state_dict": critic_sd,
    "iter": old.get("iter", 0),
    "infos": old.get("infos", {}),
}

torch.save(new, new_path)

print(f"Saved converted checkpoint to:\n{new_path}")
print("\nActor keys:")
for k in actor_sd.keys():
    print(" ", k)

print("\nCritic keys:")
for k in critic_sd.keys():
    print(" ", k)