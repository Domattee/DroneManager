# DroneControl

A package to connect to and control multiple drones.

## Installation

To install this package, simply clone this repository, move into the root directory and then install with pip:
```
pip install --upgrade pip
pip install -e .
```

As part of the installation a command called ```dm``` is installed, which starts the terminal interface. 
Alternatively you can run the app.py script.
On windows, you will also have to download the appropriate MAVSDK release (see https://mavsdk.mavlink.io/v2.0/en/cpp/guide/installation.html) and extract the mavsdk-server-bin.exe file into the same directory as drone.py.

## Usage

### Terminal interface

### API

### UAM Demo

This is a showcase demo where three drones look for a POI and start continuously observing it.
In the event we are flying with three drones, "luke", "derek" and "corran".

#### Setup
1. Boot up dm and make sure you are in the correct wifi.
2. Connect to all the drones: `connect <name>` with names "luke", "derek" or "corran"
3. Load the mission scripts: `mission-load uam`
4. Add all the drones to the mission: `uam-add <name>` IMPORTANT! The order in which the drones are added matters. The 
first drone has start position (3, -1.25), the second one (3, 0) and the third (3, 1.25). If the drones are added in the 
wrong order, they might collide during flight as their paths can cross. With `uam-status`, you can see the order of the 
drones. If drones were added in the wrong order, you can either rearrange them on the field, or remove 
`uam-remove <name>` and re-add them.
5. Check that each drone reports the correct position. If they report 0,0, the tracking system isn't connected. 
6. To send out UDP information for the visualization: `load external`

#### Mission
With the drones connected and all the scripts loaded you can begin flying missions.
1. For the single search pattern: Start the mission with `uam-singlesearch`. The first drone will fly the rectangular 
search pattern and start circling the object indefinitely once it finds it. To return it, do `uam-rtb`.
2. For the group search: Start the mission with `uam-groupsearch`. All drones will launch and fly forwards to the other 
end. The drone that finds the POI will stay with the POI, the other two come back, land and disarm. The observing 
drone starts circling until its (faked) battery runs low, when one of the drones will arm, takeoff and fly to do the 
swap. The observing drone will stop circling and wait until the swapping drone has eyes on the object, at which point 
the observing drone will rtb on its own. This swapping happens indefinitely. To stop the mission, do `uam-rtb`.
3. To fly drones from any position to their start position one-by-one you can use `uam-reset`. This should only be used 
in Gazebo, in the real demo, the drones should already be at their start positions.


# GitLab default README (check and then get rid of)

## Integrate with your tools

- [ ] [Set up project integrations](http://git.aimotion.thi.de/engel/dronecontrol/-/settings/integrations)

## Collaborate with your team

- [ ] [Invite team members and collaborators](https://docs.gitlab.com/ee/user/project/members/)
- [ ] [Create a new merge request](https://docs.gitlab.com/ee/user/project/merge_requests/creating_merge_requests.html)
- [ ] [Automatically close issues from merge requests](https://docs.gitlab.com/ee/user/project/issues/managing_issues.html#closing-issues-automatically)
- [ ] [Enable merge request approvals](https://docs.gitlab.com/ee/user/project/merge_requests/approvals/)
- [ ] [Automatically merge when pipeline succeeds](https://docs.gitlab.com/ee/user/project/merge_requests/merge_when_pipeline_succeeds.html)

## Test and Deploy

Use the built-in continuous integration in GitLab.

- [ ] [Get started with GitLab CI/CD](https://docs.gitlab.com/ee/ci/quick_start/index.html)
- [ ] [Analyze your code for known vulnerabilities with Static Application Security Testing(SAST)](https://docs.gitlab.com/ee/user/application_security/sast/)
- [ ] [Deploy to Kubernetes, Amazon EC2, or Amazon ECS using Auto Deploy](https://docs.gitlab.com/ee/topics/autodevops/requirements.html)
- [ ] [Use pull-based deployments for improved Kubernetes management](https://docs.gitlab.com/ee/user/clusters/agent/)
- [ ] [Set up protected environments](https://docs.gitlab.com/ee/ci/environments/protected_environments.html)

***

# Editing this README

When you're ready to make this README your own, just edit this file and use the handy template below (or feel free to structure it however you want - this is just a starting point!).  Thank you to [makeareadme.com](https://www.makeareadme.com/) for this template.

## Suggestions for a good README
Every project is different, so consider which of these sections apply to yours. The sections used in the template are suggestions for most open source projects. Also keep in mind that while a README can be too long and detailed, too long is better than too short. If you think your README is too long, consider utilizing another form of documentation rather than cutting out information.

## Name
Choose a self-explaining name for your project.

## Description
Let people know what your project can do specifically. Provide context and add a link to any reference visitors might be unfamiliar with. A list of Features or a Background subsection can also be added here. If there are alternatives to your project, this is a good place to list differentiating factors.

## Badges
On some READMEs, you may see small images that convey metadata, such as whether or not all the tests are passing for the project. You can use Shields to add some to your README. Many services also have instructions for adding a badge.

## Visuals
Depending on what you are making, it can be a good idea to include screenshots or even a video (you'll frequently see GIFs rather than actual videos). Tools like ttygif can help, but check out Asciinema for a more sophisticated method.

## Installation
Within a particular ecosystem, there may be a common way of installing things, such as using Yarn, NuGet, or Homebrew. However, consider the possibility that whoever is reading your README is a novice and would like more guidance. Listing specific steps helps remove ambiguity and gets people to using your project as quickly as possible. If it only runs in a specific context like a particular programming language version or operating system or has dependencies that have to be installed manually, also add a Requirements subsection.

## Usage
Use examples liberally, and show the expected output if you can. It's helpful to have inline the smallest example of usage that you can demonstrate, while providing links to more sophisticated examples if they are too long to reasonably include in the README.

## Support
Tell people where they can go to for help. It can be any combination of an issue tracker, a chat room, an email address, etc.

## Roadmap
If you have ideas for releases in the future, it is a good idea to list them in the README.

## Contributing
State if you are open to contributions and what your requirements are for accepting them.

For people who want to make changes to your project, it's helpful to have some documentation on how to get started. Perhaps there is a script that they should run or some environment variables that they need to set. Make these steps explicit. These instructions could also be useful to your future self.

You can also document commands to lint the code or run tests. These steps help to ensure high code quality and reduce the likelihood that the changes inadvertently break something. Having instructions for running tests is especially helpful if it requires external setup, such as starting a Selenium server for testing in a browser.

## Authors and acknowledgment
Show your appreciation to those who have contributed to the project.

## License
For open source projects, say how it is licensed.

## Project status
If you have run out of energy or time for your project, put a note at the top of the README saying that development has slowed down or stopped completely. Someone may choose to fork your project or volunteer to step in as a maintainer or owner, allowing your project to keep going. You can also make an explicit request for maintainers.
