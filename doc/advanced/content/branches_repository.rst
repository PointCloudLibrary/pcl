.. _branches_repository:

How to commit concurrently to trunk and branches
------------------------------------------------

Most often as a developer you will have to deal with the problem of either applying a commit to a particular branch, or pushing a modification that you just made on your machine to more than one branch. This simple example will show you one of the many ways of doing this.

The example is given for a piece of code/patch that has to be committed to both **trunk** and one of the **branches** (1.x in this case).

Before continuing, make sure your code compiles and tests carefully, and follows the indentation guidelines/C++ programming style (see :ref:`pcl_style_guide`). Then, follow this set of simple steps:

  1. In case you checked out only one of the branches or trunk, go ahead and check out **svn+ssh://svn@svn.pointclouds.org/pcl** instead. As a developer you shouldn't worry too much for the extra space that the tags/branches will consume on your hard drive, but if you do, visit **http://svn.pointclouds.org** and check out all the branches that you want to work on individually. For the former::

      svn+ssh://svn@svn.pointclouds.org/pcl pcl


    This will give you both trunk, and all the branches, and tags.

  2. Assuming that you're working in trunk, go ahead and do the following::


      cd pcl/trunk                 # change directory to where trunk is
      svn st -q                    # check to see what the commit queue contains
      svn diff . > /tmp/a.patch    # obtain a patch of your code

    At this point */tmp/a.patch* will contain the patch that has to be pushed in the repository.

  3. Change directory to the branch that you want to commit the same patch to, and do::


      cd pcl/branches/pcl-1.x      # change directory to where the branch is
      patch -p0 < /tmp/a.patch     # apply the patch

     
    One problem that we will have to deal with now is adding new files that have been added by your patch::

      svn st                       # check to see which files have "?" in the branch but should have "A" instead
      svn add <filename>           # add all files that need to be added

  4. If possible, commit both patches at the same time. Here, we're assuming that branches/pcl-1.x and trunk have the same root directory::

       cd pcl                      # change directory to where trunk+branches is
       svn st -q
       svn commit branches/ trunk/

     Alternatively, you can perform two individual commits in steps 2 and 3.


