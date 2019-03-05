##!/usr/bin/env python3
import os
import mistune

def make_markedown(directory):
    readme = os.path.join(root, "README.md")
    if not os.path.isfile(readme):
        return

    #path_to_root = os.path.relpath('.', directory)

    # Open an output file
    f = open(os.path.join(directory, "report.txt"), 'w+')

    # Add the text from the README
    readme = os.path.join(directory, "README.md")
    with open(readme, 'r') as readme_f:
        # prev_markdown = "."
        # for line in readme_f:
        #     markdown = str(mistune.markdown(line))
        #     # Get rid of paragraph markers
        #     markdown = markdown.replace('<p>','')
        #     markdown = markdown.replace('</p>','')
        #     f.write(markdown)
        #     if prev_markdown == "" and markdown == "":
        #         # Add a new line
        #         f.write("<br/><br/>")
        #     prev_markdown = markdown
        renderer = mistune.Renderer(hard_wrap=False,escape=False)
        markdown = mistune.Markdown(renderer=renderer)
        data = readme_f.read()
        f.write(markdown(data))
if __name__ == "__main__":
    for root, subdirs, files in os.walk("."):
        make_markedown(root)