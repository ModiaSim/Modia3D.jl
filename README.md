# JuliaConSubmission

This repository provides a paper for a proceeding submission at JuliaCon 2019.
For more information, go to the [proceedings website](https://proceedings.juliacon.org).

## Paper dependencies

The document can be built locally, the following dependencies need to be
installed:
- Ruby
- latexmk

## Build process

Build the paper using:
```
$ latexmk -bibtex -pdf paper.tex
```

Clean up temporary files using:
```
$ latexmk -c
```

The paper in pdf-format is provided [here](paper.pdf)

## Paper metadata

**IMPORTANT**
Some information for building the document (such as the title and keywords)
is provided through the `paper.yml` file and not through the usual `\title`
command. Respecting the process is important to avoid build errors when
submitting your work.

