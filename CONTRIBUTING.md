# Contributing to burt-csharp-examples

This guide details how to use issues and pull requests to improve burt-csharp-examples.

Please stick as close as possible to the guidelines. That way we ensure quality in our products and merge requests become easier.

For our standard Software Version Control Procedures at Barrett please see:

- `\\fs2\projects\Barrett Medical\Standards\Software Standards\SWSTD-1_RevAH_Git Version Control Process_RELEASED.doc`

## Merge requests

### Naming and Descriptions

Merge request names should be descriptive, rather than using with the default "Merging branch-name into branch name" name one should fill in details using the following format.

- [Project Name] Brief description of change or Issue #s
- [Common] Brief description of changes.

In the case of a documentation update:

- [Documentation] Brief  description.

In the case of an added script or change to shared files:

- [Infrastructure] Brief description.

The brief should give a good idea of what has been done so
code reviewers know what to focus on. The description that follows should include Issues #s addressed in the change along with a more detailed description. Write your descriptions for your audience, the code reviewers. Make sure they know what to look for, and where necessary reference Issues, SOPs, CRs, Meeting Minutes to build the story of the merge request. It is not necessary to go into excruciating detail, but it is useful to have enough detail to make the code review go smoothly.

Merge requests should also get due dates, this will help code reviewers prioritize there work and get to your Merge Request in a timely, but not rushed manner.

If you do not want code review or a merge to happen yet mark your merge requests as Works in Progress by changing the name to:

- WIP: Merge Request Name

### Code Review

To initiate a code review put the following list in your merge requests:

```
Reviewers:
- [ ] @heather
- [ ] @cw
- [ ] @amyblank
- [ ] @bz
- [ ] @jchung
- [ ] @ps
```

**How many reviewers do I need?**

For non-trivial changes to code you need, at least _3_ individuals (including yourself) to review the code and provide you with a check before merging the merge request. However, for trivial changes (typo corrections), documentation updates, or infrastructure changes (changes to scripts, etc.) you only need _2_ reviewers (including yourself).

**When do we code review?**

Code reviews are only required when merging into devel or master, but it is encouraged to do smaller code reviews as developers merge features into each others branches or even their own.

**How do we code review?**
Code reviewers should file their comments inline and also give a Merge Request wide code review.

Any comment which is not dismissed or resolved in the merge request should get Issues and be attached to a milestone when appropriate.

**How does a reviewer claim a code review?**

They edit the merge request and put - WIP next to their name, for example:

```
- [ ] @cw - WIP
```

**What is the format for a merge request?**

Please see [the template](.gitlab/merge_request_templates/code-review.md)
