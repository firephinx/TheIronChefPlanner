# TheIronChefPlanner

## Installation Instructions
1. Clone this repository into your catkin workspace.
2. Run `catkin_make` in your catkin workspace.
3. Source your `devel/setup.bash` in your catkin_workspace.
4. Install aws-cli following instructions here: `https://docs.aws.amazon.com/cli/latest/userguide/awscli-install-bundle.html`
5. Generate a key and give it full DynamoDB access.
6. Configure aws-cli following instructions here: `https://docs.aws.amazon.com/cli/latest/userguide/cli-chap-getting-started.html`
7. Download and install pip following instructions here: `https://pip.pypa.io/en/stable/installing/`
8. Install testresources by typing: `pip install testresources --user`
9. Install boto3 by typing: `pip install boto3 --user`

## Runtime Instructions
1. To begin, type `rosrun theironchef_planner theironchef_main.py` into a terminal.