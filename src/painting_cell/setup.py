import os
from glob import glob
from setuptools import setup

package_name = 'painting_cell'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        (os.path.join('share', package_name, 'meshes'), glob('meshes/*')),
        (os.path.join('share', package_name, 'paths'), glob('paths/*')),
        (os.path.join('share', package_name, 'gz'), glob('gz/*.*')),
        (os.path.join('share', package_name, 'gz', 'meshes'), glob('gz/meshes/*')),
        (os.path.join('share', package_name, 'gz', 'paint'), glob('gz/paint/*')),
        (os.path.join('share', package_name, 'gz', 'kuka'), glob('gz/kuka/*')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Assaad Mahmoudi',
    maintainer_email='assaadmahmoudi0@gmail.com',
    description='Robotic car painting cell simulation',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'plan_paint = painting_cell.plan_paint:main',
            'play_paint = painting_cell.play_paint:main',
            'gz_play = painting_cell.gz_play:main',
            'scene_setup = painting_cell.scene_setup:main',
        ],
    },
)
