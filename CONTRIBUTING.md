# Contributing to PCL

Please take a moment to review this document in order to make the contribution
process easy and effective for everyone involved.

Following these guidelines helps to communicate that you respect the time of
the developers managing and developing this open source project. In return,
they should reciprocate that respect in addressing your issue or assessing
patches and features.


## Using the issue tracker

The [issue tracker](https://github.com/PointCloudLibrary/pcl/issues) is
the preferred channel for [bug reports](#bugs) and
[submitting pull requests](#pull-requests), but please respect the following
restrictions:

* Please **do not** use the issue tracker for personal support requests (use
  [mailing list](http://www.pcl-users.org/)).

* Please **do not** derail or troll issues. Keep the discussion on topic and
  respect the opinions of others.


<a name="bugs"></a>
## Bug reports

A bug is a _demonstrable problem_ that is caused by the code in the repository.
Good bug reports are extremely helpful - thank you!

Guidelines for bug reports:

1. **Check if the issue has been reported** &mdash; use GitHub issue search and
   mailing list archive search.

2. **Check if the issue has been fixed** &mdash; try to reproduce it using the
   latest `master` branch in the repository.

3. **Isolate the problem** &mdash; ideally create a reduced test
   case.

A good bug report shouldn't leave others needing to chase you up for more
information. Please try to be as detailed as possible in your report. What is
your environment? What steps will reproduce the issue? What would you expect to
be the outcome? All these details will help people to fix any potential bugs.

Example:

> Short and descriptive example bug report title
>
> A summary of the issue and the OS environment in which it occurs. If
> suitable, include the steps required to reproduce the bug.
>
> 1. This is the first step
> 2. This is the second step
> 3. Further steps, etc.
>
> Any other information you want to share that is relevant to the issue being
> reported. This might include the lines of code that you have identified as
> causing the bug, and potential solutions (and your opinions on their
> merits).


<a name="pull-requests"></a>
## Pull requests

Good pull requests - patches, improvements, new features - are a fantastic
help. They should remain focused in scope and avoid containing unrelated
commits.

**Please ask first** before embarking on any significant pull request (e.g.
implementing features, refactoring code), otherwise you risk spending a lot of
time working on something that the project's developers might not want to merge
into the project. Please read the [tutorial on writing a new PCL class](http://pointclouds.org/documentation/tutorials/writing_new_classes.php#writing-new-classes) if you want to contribute a
brand new feature.

<a name="checklist"></a>
### Checklist

Please use the following checklist to make sure that your contribution is well
prepared for merging into PCL:

1. Source code adheres to the coding conventions described in [PCL Style Guide](http://pointclouds.org/documentation/advanced/pcl_style_guide.php).
   But if you modify existing code, do not change/fix style in the lines that
   are not related to your contribution.

2. Commit history is tidy (no merge commits, commits are [squashed](http://davidwalsh.name/squash-commits-git)
   into logical units).

3. Each contributed file has a [license](http://pointclouds.org/documentation/tutorials/writing_new_classes.php#licenses) text on top.

### Suggested process

Adhering to this process is the best way to get your work included in the
project:

1. [Fork](http://help.github.com/fork-a-repo/) the project, clone your fork,
   and configure the remotes:

   ```bash
   # Clone your fork of the repo into the current directory
   git clone https://github.com/<your-username>/pcl.git
   # Navigate to the newly cloned directory
   cd pcl
   # Assign the original repo to a remote called "upstream"
   git remote add upstream https://github.com/PointCloudLibrary/pcl.git
   ```

2. If you cloned a while ago, get the latest changes from upstream:

   ```bash
   git checkout master
   git pull upstream master
   ```

3. Create a new topic branch (off the main project development branch) to
   contain your feature, change, or fix:

   ```bash
   git checkout -b <topic-branch-name>
   ```

4. Commit your changes in logical chunks. For any Git project, some good rules
   for commit messages are

   * the first line is commit summary, 50 characters or less,
   * followed by an empty line
   * followed by an explanation of the commit, wrapped to 72 characters.

   See [a note about git commit messages](http://tbaggery.com/2008/04/19/a-note-about-git-commit-messages.html)
   for more.

   The first line of a commit message becomes the **title** of a pull request on
   GitHub, like the subject line of an email. Including the key info in the
   first line will help us respond faster to your pull.

   If your pull request has multiple commits which revise the same lines of
   code, it is better to [squash](http://davidwalsh.name/squash-commits-git)
   those commits together into one logical unit.

   But you don't always have to squash -- it is fine for a pull request to
   contain multiple commits when there is a logical reason for the separation.

5. Locally merge (or rebase) the upstream development branch into your topic
   branch:

   ```bash
   git pull [--rebase] upstream master
   ```

6. Push your topic branch up to your fork:

   ```bash
   git push origin <topic-branch-name>
   ```

7. [Open a Pull Request](https://help.github.com/articles/using-pull-requests/)
    with a clear title and description.

8. After your Pull Request is away, you might want to get yourself back onto
   `master` and delete the topic branch:

   ```bash
   git checkout master
   git branch -D <topic-branch-name>
   ```

**IMPORTANT**: By submitting a patch, you agree to allow the project owners to
license your work under the the terms of the [BSD License](LICENSE.txt).
