GEN = gen
BUILD = build

$(GEN):
	@mkdir -p $@

gen-prepare: | $(GEN)
	@cp code-generator/CMakeLists.txt $(GEN)/CMakeLists.txt
	@cp thirdparty/orocos-kdl/chainhdsolver_vereshchagin_fext.hpp $(GEN)/chainhdsolver_vereshchagin_fext.hpp
	@cp thirdparty/orocos-kdl/chainhdsolver_vereshchagin_fext.cpp $(GEN)/chainhdsolver_vereshchagin_fext.cpp
	@cp thirdparty/kinova/GEN3_URDF_V12.urdf $(GEN)/GEN3_URDF_V12.urdf

gen-ir-to-code:
	@stst -s "<>" -t code-generator tmpl.application $(GEN)/ir.json > $(GEN)/main.cpp

gen-comp:
	@cmake -S $(GEN) -B $(GEN)/build -DCMAKE_BUILD_TYPE=Debug
	@cd $(GEN)/build && make

gen-ir-sc0a:
	@python ir_gen.py models/sc0a-right-arm.json > $(GEN)/ir.json

gen-ir-sc0b:
	@python ir_gen.py models/sc0b-dual-arm.json > $(GEN)/ir.json

gen-ir-sc1:
	@python ir_gen.py models/sc1.json > $(GEN)/ir.json

gen-ir-sc2:
	@python ir_gen.py models/sc2.json > $(GEN)/ir.json

sc0a: gen-prepare gen-ir-sc0a gen-ir-to-code gen-comp
sc0b: gen-prepare gen-ir-sc0b gen-ir-to-code gen-comp
sc1: gen-prepare gen-ir-sc1 gen-ir-to-code gen-comp
sc2: gen-prepare gen-ir-sc2 gen-ir-to-code gen-comp


tutorial-html:
	@sphinx-build -M html docs/sphinx/source/ build/

tutorial-live:
	@sphinx-autobuild docs/sphinx/source/ build/

tutorial-pdf:
	@sphinx-build -M latexpdf docs/sphinx/source/ build/


check:
	python check.py models/sc0a-right-arm.json
	python check.py models/sc0b-dual-arm.json
	python check.py models/sc1.json
	python check.py models/sc2.json

count:
	python count.py models/sc0a-right-arm.json
	python count.py models/sc0b-dual-arm.json
	python count.py models/sc1.json
	python count.py models/sc2.json

clean:
	@rm -rf $(GEN) $(BUILD)
