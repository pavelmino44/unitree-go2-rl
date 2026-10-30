from setuptools import setup

package_name = "go2_eval"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml", "README.md"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Pavel",
    maintainer_email="pavelmino44@gmail.com",
    description="Evaluation of Go2 policies in MuJoCo on the unitree_rl_lab scenarios (sim-to-sim).",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "evaluate = go2_eval.evaluate:main",
            "eval_report = go2_eval.eval_report:main",
            "eval_compare = go2_eval.compare:main",
        ],
    },
)
