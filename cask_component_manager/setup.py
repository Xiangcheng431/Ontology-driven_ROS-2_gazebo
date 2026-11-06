from setuptools import find_packages, setup
from glob import glob
package_name = 'cask_component_manager'
setup(name=package_name, version='0.0.0', packages=find_packages(exclude=['test']), data_files=[('share/ament_index/resource_index/packages', ['resource/' + package_name]), ('share/' + package_name, ['package.xml']), ('share/' + package_name + '/config', glob('config/*.json'))], install_requires=['setuptools'], zip_safe=True, maintainer='xiangcheng', maintainer_email='xiangcheng@todo.todo', description='Ontology-driven component state and RViz visualization manager', license='Apache-2.0', extras_require={'test': ['pytest']}, entry_points={'console_scripts': ['component_manager_node = cask_component_manager.component_manager_node:main']})
