from setuptools import setup

setup(
    name="cask_handoff",
    version="0.1.0",
    packages=["cask_handoff"],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/cask_handoff"]),
        ("share/cask_handoff", ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Xiangcheng Li",
    maintainer_email="xiangcheng@todo.todo",
    description="Coordinated physical cargo handoff protocol",
    license="Apache-2.0",
)
