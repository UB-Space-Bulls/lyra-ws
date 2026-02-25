from setuptools import setup
import os
from glob import glob

package_name = 'urdf_description'

# Build recursive data_files for worlds + meshes
data_files = []

for f in glob('worlds/**/*', recursive=True):
    if os.path.isfile(f):
        rel_path = os.path.relpath(os.path.dirname(f), 'worlds')
        target = os.path.join('share', package_name, 'worlds', rel_path)
        data_files.append((target, [f]))

# Add other non-worlds files
data_files.extend([
    ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
    (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
    (os.path.join('share', package_name, 'config'), glob('config/*')),
    (os.path.join('share', package_name, 'meshes'), glob('meshes/*.stl')),
])

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='author',
    maintainer_email='todo@todo.com',
    description='The ' + package_name + ' package',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)