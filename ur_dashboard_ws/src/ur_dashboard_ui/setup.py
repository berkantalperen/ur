from setuptools import setup

package_name = "ur_dashboard_ui"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", ["launch/ur_dashboard_ui.launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="OpenAI",
    maintainer_email="dev@openai.com",
    description="Qt-based dashboard UI for discovering and calling UR dashboard services.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "dashboard_ui = ur_dashboard_ui.dashboard_ui:main",
        ],
    },
)
