#!/bin/bash

# this will generate a document with question marks in place of unknown references
pdflatex ex.tex

# this will parse all the .bib files that were included in the article and generate metainformation regarding references
bibtex ex

# this will generate document with all the references in the correct places
pdflatex ex.tex

echo "The pdf file ex.pdf was created."
