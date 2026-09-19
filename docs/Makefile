# Documentation build.
#
#   make            regenerate facts + diagrams, then build the PDF
#   make diagrams   Draw.io files and figure PDFs only
#   make facts      generated_facts.tex only
#   make clean      remove LaTeX intermediates (keeps the PDF)
#   make distclean  also remove the PDF, figures and diagrams
#
# Facts and figures are BUILD PRODUCTS derived from the robot model. Never edit
# generated_facts.tex or docs/figures/*.pdf by hand; change the model or
# tools/make_diagrams.py and rebuild.

TOOLS  := ../tools
TEX    := robodog_architecture.tex
PDF    := $(TEX:.tex=.pdf)
MODEL  := ../ros2_ws/src/robodog_description/config/robot_parameters.yaml
ACT    := ../ros2_ws/src/robodog_description/config/robstride02.yaml
FIGS   := $(patsubst %,figures/%.pdf,system_architecture software_architecture \
            node_topic_graph control_architecture data_flow hardware_boundary \
            state_machine domain_model simulation_architecture perception_pipeline)

.PHONY: all facts diagrams clean distclean
all: $(PDF)

$(PDF): $(TEX) preamble.tex generated_facts.tex $(FIGS)
	latexmk -pdf -interaction=nonstopmode -halt-on-error $(TEX)

generated_facts.tex: $(TOOLS)/make_doc_facts.py $(MODEL) $(ACT)
	python3 $(TOOLS)/make_doc_facts.py

facts: generated_facts.tex

$(FIGS): $(TOOLS)/make_diagrams.py $(TOOLS)/diagram_lib.py
	python3 $(TOOLS)/make_diagrams.py

diagrams: $(FIGS)

clean:
	latexmk -c $(TEX)

distclean: clean
	rm -f $(PDF) generated_facts.tex figures/*.pdf diagrams/*.drawio
