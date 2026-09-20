# control-base — the product-neutral base: the Plant contract, the contact model and the efference copy.
#
#   make test          # this crate's suite, then the comment rules over the tree
#   make comments      # the comment rules alone, with the local approximation for the rest
#
# Nothing in `src/` names the comment rules; they are a dev-dependency, so the base's own links are
# unchanged by this target.
#
# Every Cargo command below is run as `mbx <subcommand>` (the build-cache wrapper). The rules live in
# ../comment-why, read text, and need no toolchain of their own: the gate inside `mbx test` is
# tests/comment_why.rs, and `make comments` is the same rules over the working tree, with that crate's
# local approximation for the comments the rules cannot decide.

COMMENT_WHY ?= ../comment-why

.PHONY: test comments

test:
	mbx test
	$(MAKE) comments

comments:
	mbx run --quiet --manifest-path $(COMMENT_WHY)/Cargo.toml --bin comment-why -- --review
