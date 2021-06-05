:tocdepth: 2

Getting Started
===============

Installation
------------

.. _Github Page: https://github.com/tmigimatsu/spatial-dyn

See the `GitHub page`_ for installation instructions.

Loading Articulated Bodies with URDFs
-------------------------------------

URDF specifications get loaded into an ``ArticulatedBody`` object. This object
is the central data structure of `spatial-dyn`.

C++
^^^

.. code-block:: cpp

   #include <spatial_dyn/spatial_dyn>

   namespace dyn = spatial_dyn;

   int main(int argc, char* argv[]) {
     // Load URDF model.
     dyn::ArticulatedBody ab(dyn::urdf::LoadModel("resources/robot.urdf"));

     // Set robot state.
     Eigen::VectorXd q;
     q << 0., 0., 0., 0., 0., 0.;
     ab.set_q(q);

     return 0;
   }

See the C++ :spatial_dyn:`spatial_dyn::ArticulatedBody` reference for more details.

Python
^^^^^^

.. code-block:: python

   import numpy as np
   import spatialdyn as dyn

   # Load URDF model.
   ab = dyn.ArticulatedBody(dyn.urdf.load_model("resources/robot.urdf"))

   # Set robot state.
   ab.q = np.zeros((6,))

See the Python :class:`spatialdyn.ArticulatedBody` reference for more details.
