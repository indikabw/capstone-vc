import subprocess
import os
from ament_index_python.packages import get_package_share_directory

def test_behave_features():
    pkg_dir = get_package_share_directory('custom_bot_reasoning')
    features_dir = os.path.join(pkg_dir, 'features')
    
    result = subprocess.run(['python3', '-m', 'behave', features_dir], capture_output=True, text=True)
    print(result.stdout)
    assert result.returncode == 0, f"Behave tests failed:\n{result.stderr}"
