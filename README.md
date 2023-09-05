Clone: 
```
git clone git@github.com:wpi-huron/huron.git --recurse-submodules
```

Build:
```
bazel build <target> [--verbose_failures] --action_env=HURON_DIR=$(pwd)
```

Note: ```HURON_DIR``` needs to be set to the root of this repo.
