Clone the main repo and all submodules: 
```
git clone git@github.com:wpi-huron/huron.git --recurse-submodules
```

Build:
1. Make sure you are in the root of this repo (`huron/`)
2. Build the repo
```
bazel build <target> [--verbose_failures] --action_env=HURON_DIR=$(pwd)
```

Note: ```HURON_DIR``` needs to be set to the root of this repo - that's why `--action_env=HURON_DIR=$(pwd)` is important.
