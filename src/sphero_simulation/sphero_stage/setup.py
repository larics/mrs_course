from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'sphero_stage'

def generate_data_files(src_dir, dest_prefix):
    """Recursively find all files in src_dir and create data_files entries."""
    data_files = []
    if os.path.exists(src_dir):
        for root, dirs, files in os.walk(src_dir):
            if files:  # Only add directories that contain files
                # Convert source path to destination path
                rel_path = os.path.relpath(root, '.')
                dest_path = os.path.join(dest_prefix, rel_path)
                file_paths = [os.path.join(root, f) for f in files]
                data_files.append((dest_path, file_paths))
    return data_files

# Generate data files - this will automatically include all files recursively
config_data_files = generate_data_files('config', f'share/{package_name}')
launch_data_files = generate_data_files('launch', f'share/{package_name}')

setup(
    name=package_name,
    version='2.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ] + config_data_files + launch_data_files,
    install_requires=['setuptools', 'pyyaml'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='marko.krizmancic@fer.hr',
    description='A simple package for Sphero robot simulation in Stage simulator',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'robot_controller = sphero_stage.robot_controller:main',
        ],
    },
)
