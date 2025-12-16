# ================================
#  TRUE CROSS-PLATFORM MAKEFILE
# ================================

NAME = Robotics IK/FK System
REQ  = requirements.txt
SRC  = main.py

# Detect OS
OS := $(shell uname 2>/dev/null)

# ================================
#  WINDOWS CONFIG
# ================================
ifeq ($(OS),)
	VENV = .venv
	PY   = $(VENV)/Scripts/python
	PIP  = $(VENV)/Scripts/pip
	RM   = rmdir /s /q
	CACHE_RM = rmdir /s /q
	NULLDEV = nul
	MAKE_VENV = python -m venv $(VENV)
else
# ================================
#  LINUX / WSL CONFIG
# ================================
	VENV = .venv
	PY   = $(VENV)/bin/python
	PIP  = $(VENV)/bin/pip
	RM   = rm -rf
	CACHE_RM = rm -rf
	NULLDEV = /dev/null
	MAKE_VENV = python3 -m venv $(VENV)
endif

# ================================
#  COLORS (WSL/LINUX ONLY — harmless on Windows)
# ================================

GREEN  = \033[1;32m
RED    = \033[1;31m
BLUE   = \033[1;34m
YELLOW = \033[1;33m
CYAN   = \033[1;36m
RESET  = \033[0m

# ================================
#  DEFAULT TARGET
# ================================

all: banner run

# ================================
#  BANNER
# ================================

banner:
	@echo "$(CYAN)"
	@echo "==========================================="
	@echo "     🚀  $(NAME)"
	@echo "==========================================="
	@echo "$(RESET)"

# ================================
#  VIRTUAL ENV
# ================================

venv:
	@echo "$(BLUE)[CREATE] Virtual Environment...$(RESET)"
	@$(MAKE_VENV)
	@$(PIP) install --upgrade pip
	@echo "$(GREEN)[OK] VENV Ready$(RESET)"

# ================================
#  INSTALL LIBRARIES
# ================================

install:
	@echo "$(BLUE)[INSTALL] Dependencies...$(RESET)"
	@$(PIP) install -r $(REQ)
	@echo "$(GREEN)[OK] All Packages Installed$(RESET)"

# ================================
#  RUN PROJECT
# ================================

run:
	@echo "$(YELLOW)[RUN] Launching Application...$(RESET)"
	@$(PY) $(SRC)
	@echo "$(YELLOW)[RUN] Application is Finished $(RESET)"

# ================================
#  CLEAN CACHE
# ================================

clean:
	@echo "$(RED)[CLEAN] Removing Cache Files...$(RESET)"
	@$(CACHE_RM) __pycache__ 2>$(NULLDEV) || true
	@$(CACHE_RM) gui/__pycache__ 2>$(NULLDEV) || true
	@$(CACHE_RM) manipulators/__pycache__ 2>$(NULLDEV) || true
	@echo "$(GREEN)[OK] Clean Done$(RESET)"

# ================================
#  FULL RESET
# ================================

fclean: clean
	@echo "$(RED)[RESET] Removing Virtual Environment...$(RESET)"
	@$(RM) $(VENV) 2>$(NULLDEV) || true
	@echo "$(GREEN)[OK] Full Reset Complete$(RESET)"

# ================================
#  REBUILD
# ================================

re: fclean venv install

# ================================
#  PHONY
# ================================

.PHONY: all banner venv install run clean fclean re
