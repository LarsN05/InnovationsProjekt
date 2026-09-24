#!/usr/bin/env python3
import argparse
import pathlib
import re
import sys
import minify_html

TEMPLATE = """#ifndef {guard}
#define {guard}

const char htmlPage[] PROGMEM = R"rawliteral(
{content}
)rawliteral";

#endif
"""

def make_guard(name: str) -> str:
    s = re.sub(r'[^A-Za-z0-9]', '_', name).upper()
    if not s.endswith('_H'):
        s = s + '_H'
    return s


def main():
    p = argparse.ArgumentParser(
        description="Embed an HTML file into a C header with R\"rawliteral(...)\""
    )
    p.add_argument("input_html", type=pathlib.Path, help="input HTML file (websocket.html)")
    p.add_argument("output_h", type=pathlib.Path, help="output header .h file (websocket_html.h)")
    p.add_argument(
        "--no-minify",
        dest="minify",
        action="store_false",
        help="disable HTML minification (enabled by default)"
    )
    args = p.parse_args()

    if not args.input_html.exists():
        print("Input file does not exist:", args.input_html, file=sys.stderr)
        sys.exit(2)

    content = args.input_html.read_text(encoding="utf-8")

    if args.minify:
        try:
            length_before = len(content)
            content = minify_html.minify(
                content,
                minify_css=True,
                minify_js=True
            )
            length_after = len(content)
            print(f"HTML minified: {length_before} -> {length_after} bytes")
        except Exception as e:
            print(f"Warning: HTML minification failed: {e}", file=sys.stderr)

    # Raw string delimiter safety check
    if ')rawliteral' in content:
        print(
            "ERROR: input contains the sequence ')rawliteral' which would break the raw literal delimiter.",
            file=sys.stderr
        )
        sys.exit(3)

    guard = make_guard(args.output_h.name)
    out = TEMPLATE.format(guard=guard, content=content.rstrip())

    args.output_h.parent.mkdir(parents=True, exist_ok=True)
    args.output_h.write_text(out + "\n", encoding="utf-8")
    print("Wrote", args.output_h)


if __name__ == "__main__":
    main()
