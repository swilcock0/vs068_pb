# PyBullet demo - VS068

![Robot demo](resources/image.png?raw=true)

## Initial install

To run, first install [Anaconda](https://www.anaconda.com/)

Then, in a conda prompt, we're going to create a new virtual python environment and install pybullet (and compas_fab) to it. Type  
```bash
conda config --add channels conda-forge
conda create -n my-project compas_fab spyder pybullet
```


Now, every time you open the Anaconda prompt, activate your environment using
```bash
activate my-project
```

## Running the Spyder python IDE
Having activated the shiny new environment, we can run the Spyder scientific integrated development environment (IDE). Simply Type
```
spyder
```

Once open, load the "arm_pybullet.py" file. This can now be run to produce the VS068 demo. 


## Expanding
See the [pyBullet quickstart guide](https://docs.google.com/document/d/10sXEhzFRSnvFcl3XxNGhnD4N2SedqwdAvK3dsihxVUA/edit#heading=h.2ye70wns7io3) for info on how to use the API. Of particular interest may be the section on calculateInverseKinematics (to find the joint values to reach a specific end effector point).

Additionally, for Grasshopper/COMPAS_FAB integration, [these documents](https://gramaziokohler.github.io/compas_fab/latest/examples/05_backends_pybullet.html#examples-pybullet) are instructive. The command 
```bash
python -m compas_rhino.install
```
should make compas available to Grasshopper.