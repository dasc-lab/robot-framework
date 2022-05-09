---
layout: default
title:  "How To: Documentation"
date:   2022-05-09
math: katex
has_children: False
nav_order: 2
---

# Documentation

How to update and build the documentation.

References:
1. https://just-the-docs.github.io/just-the-docs/
2. https://just-the-docs.github.io/just-the-docs/docs/index-test/
3. https://pdmosses.github.io/just-the-docs-tests/
4. https://pdmosses.github.io/test-nav/




## Updating: 
1. Clone the repo locally
2. Make the changes to the required markdown files
3. Push to github's master branch. 

The website should get updated soon enough. 

## Testing Documentation Locally:
Sometimes, it is convenient to test the documentation locally, and then push it all at once. 

To do this, after cloning the repo, run

```
bundle exec jekyll serve --livereload --port 4001
```

and navigate to:
```
http://localhost:4001/
```

the website will reload as you save changes to underlying files. 


## Folder Structure:
There is  (at the time of writing) a folder structure like this:
```
.
├── 404.html                     ## this is displayed with a 404 error [no need to touch]
├── Gemfile                      ## defines some of the packages needed to build this website, [no need to touch]
├── _config.yml                  ## some baseline configurations [no need to touch]
├── _site                        ## local build of website goes here, [no need to touch, will be auto-updated]
│   └── *lots of files*
├── assets                       
│   └── images                   ## all images need to go into this folder
├── documentation_guide          ## make a folder for each subsection we want
│   └── documentation.md
├── favicon.ico
├── index.md                     ## first displayed page
├── px4_robots                  
│   ├── first_experiment.md      ## create individual documentation pages here
│   ├── px4_robots.md            ## this file is the section header
│   ├── px4_setup.md             ## individual documentation here
│   └── raspi_setup.md    
└── readme.md
```