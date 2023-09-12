Clone the main repo and all submodules: 
```
git clone git@github.com:wpi-huron/huron.git --recurse-submodules
```

Build:
1. Make sure you are in the root of this repo (`huron/`)
2. Build the repo
```
bazel build <target> [--verbose_failures] [--config=<config_name>] --action_env=HURON_DIR=$(pwd)
```

Notes: 

- `HURON_DIR` needs to be set to the root of this repo - that's why `--action_env=HURON_DIR=$(pwd)` is important.
- To build on Linux x86_64 machines, set `--config=x86_64`. To build on arm64 machines like the Raspberry Pi, set `--config=arm64`.
