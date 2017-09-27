#!/bin/bash

# this will generate a document with question marks in place of unknown references
pdflatex ex1.tex

# this will parse all the .bib files that were included in the article and generate metainformation regarding references
bibtex ex1

# this will generate document with all the references in the correct places
pdflatex ex1.tex

echo "The pdf file ex1.pdf was created."
