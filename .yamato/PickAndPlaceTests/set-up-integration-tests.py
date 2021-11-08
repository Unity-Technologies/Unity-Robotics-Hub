import os
import shutil
import glob
from collections import OrderedDict
from unityparser import UnityDocument

# NOTE: This must match the flag defined in the Unity Integration Tests
_INTEGRATION_TEST_DEFINE = "INTEGRATION_TEST"

# This script is executed inside of a bokken image in order to automate the manual steps a user would
# perform when going through the tutorials. This allows us to perform integration tests on the expected final
# state of a tutorial project.


script_dir = os.path.dirname(os.path.realpath(__file__))
root_dir = os.path.join(script_dir, "..", "..", "tutorials", "pick_and_place")
external_scripts_dir = os.path.join(root_dir, "Scripts")
project_dir = os.path.join(root_dir, "PickAndPlaceProject")
project_scripts_dir = os.path.join(project_dir, "Assets", "Scripts")
external_ros_scripts_dir = os.path.join(script_dir, "..", "..", "tutorials", "ros_unity_integration", "unity_scripts")
# project_settings_file = os.path.join(project_dir, "ProjectSettings", "ProjectSettings.asset")

scripts_to_move = glob.glob(os.path.join(external_scripts_dir, "*.cs"))
for external_script in scripts_to_move:
    script_name = os.path.basename(external_script)
    script_destination = os.path.join(project_scripts_dir, script_name)
    print(f">>> Copying {external_script} to {script_destination}")
    shutil.copyfile(external_script, script_destination)

scripts_to_move = glob.glob(os.path.join(external_ros_scripts_dir, "*.cs"))
for external_script in scripts_to_move:
    script_name = os.path.basename(external_script)
    script_destination = os.path.join(project_scripts_dir, script_name)
    print(f">>> Copying {external_script} to {script_destination}")
    shutil.copyfile(external_script, script_destination)

files_to_cat = []
message_dir = os.path.join(project_dir, "Assets", "RosMessages")
print(f">>> Files in {message_dir}:")
for root, _, files in os.walk(message_dir):
    level = root.replace(message_dir, '').count(os.sep)
    indent = ' ' * 4 * level
    print('{}{}/'.format(indent, os.path.basename(root)))
    subindent = ' ' * 4 * (level + 1)
    for f in files:
        print('{}{}'.format(subindent, f))
        if f.endswith(".cs") or f.endswith(".asmdef"):
            files_to_cat.append(os.path.join(root, f))

# On Yamato, Unity fails to recompile the message directory under mysterious circumstances, so moving it into
# the Scripts directory to attempt to force a recompile
print(f"Moving {message_dir} to {project_scripts_dir}")
shutil.move(message_dir, project_scripts_dir)

# for f in files_to_cat:
#     print(f">>> {f}:")
#     os.system(f"cat {f}")
#     print("\n")

# We must keep a backup copy of the ProjectSettings.asset because UTR will serialize the one in the project as a binary
# file, making it impossible to load as a yaml file here
test_settings = os.path.join(".", ".yamato", "PickAndPlaceTests", "IntegrationTestSettings.asset")
project_settings_relative = os.path.join(".", "tutorials", "pick_and_place", "PickAndPlaceProject", "ProjectSettings", "ProjectSettings.asset")
shutil.copyfile(test_settings, project_settings_relative)

# TODO: We have to use the above, bad solution, for now, because the below solution won't work until
#       UTR stops re-serializing the settings
# project_settings_asset = UnityDocument.load_yaml(project_settings_file)
# scripting_defines = project_settings_asset.entry.scriptingDefineSymbols  # type: OrderedDict
# if scripting_defines[1]:
#     scripting_defines[1] += f";{_INTEGRATION_TEST_DEFINE}"
# else:
#     scripting_defines[1] = _INTEGRATION_TEST_DEFINE
# project_settings_asset.dump_yaml()
