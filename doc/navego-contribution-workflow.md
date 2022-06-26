
# NaveGo contribution workflow

The main idea is to follow the [github worflow](https://docs.github.com/en/get-started/quickstart/github-flow). There is nothing new about our methodology, but since the Github worflow document is large, next a summary is provided.

1. We work hierarchically with 2 branches simultaneously:

    `main`: Production branch. Nothing is merged to `main` except there is a strong consensus among all the participants working on the repository. Only the `develop` branch can be merged to the `main` branch.

    `develop`: it is the integration branch used by all the branches to merge particular changes.

2. For each new feature you need to create a new topic branch. These branches should always be created from `develop`. The process is the following:

    a. Download the complete repository: `git clone htpp://github.com/<user>/<repository>.git`

    b. Change to branch develop: `git checkout develop`.

    c. Get the last version from origin: `git pull origin develop`.

    d.  Create a new branch from develop and change to that new branch: `git checkout -b <branch-name> develop`. 

    e. Then you work on the new branch adding a new feature or fixing a bug.

    f. Add the files that have to be commited `git add <file1> <file2> ...`.

    g. Commit changes in your local repo: `git commit -m "<a descriptive comment about the commit...>"`. In the comment you can make a link to the related issue using #<issue-number>, or close an issue directly from the comment. For example, `git commit -m "Minor corrections, closes #9"`. For more information about this topic, check out this [link](https://github.com/gitbucket/gitbucket/wiki/How-to-Close-Reference-issues-and-pull-request) and this  [link](https://stackoverflow.com/questions/60027222/github-how-can-i-close-the-two-issues-with-commit-message).

    h. Submit the branch content to the remote repo `origin`, creating the branch if it doesn't exist or updating it if it already exists: `git push origin <branch-name>`.

    i. Then, you must create a Pull Request (PR) at the github site. Comments from the others developers are expected.

    j. When the PR is approved, the merge between `develop` and `<branch-name>` is made at the github site.

    k. Finally, if all issues were resolved related the branch, the branch may be deleted, `git branch -d <branch>`.

## Downloading other branches

Later, if someone creates a new branch and sets it to `origin`, and you want to download it to your local repo, you first download a snapshot of the entire project doing `git fetch origin`. Then, you can do `git checkout <branch-name>` or alternatively do `git checkout --track origin / <branch-name>`


## About the releases

A release is an official version of the project under development. When you are going to make a release, you typically do a single PR from `develop` to `main`, check and merge in `main`. 

Then, the version history is usually keeped by adding tags in `main` (you can add them in any branch anyway if you need them). The tags are like markers in the history and then if necessary you can create branches off of those tags.

## Useful commands

### Check current branch and commited files
`git status`

### Delete N commits
`git reset --hard HEAD~2` command moves the current branch backward by 2 commits.

### Unstage a file (uncommit)
`git restore --staged <file>...`

### Change to any branch 
`git checkout <branch-name>`.
 
