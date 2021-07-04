BUILD=build
APPS=OFF
CMAKE_ARGS:=$(CMAKE_ARGS)

all:
	@mkdir -p $(BUILD)
	@cd $(BUILD); cmake .. -DBUILD_APPS=$(APPS) -DCMAKE_BUILD_TYPE=Release $(CMAKE_ARGS) && $(MAKE)

app:
	@$(MAKE) all APPS=ON

clean:
	@rm -rf $(BUILD)
