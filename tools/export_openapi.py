#!/usr/bin/env python3
"""Export the schema from the tested service with its installed dependencies."""
from __future__ import annotations

import argparse
import json
from pathlib import Path
from urllib.request import urlopen


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--base-url', required=True)
    parser.add_argument('--output', type=Path, default=Path('docs/openapi.json'))
    args = parser.parse_args()
    with urlopen(args.base_url.rstrip('/') + '/openapi.json', timeout=30) as response:
        schema = json.load(response)
    if not isinstance(schema.get('paths'), dict) or not schema.get('openapi'):
        raise ValueError('Service response is not an OpenAPI schema')
    if any(path.startswith('/api/') and not path.startswith('/api/v2/') for path in schema['paths']):
        raise ValueError('Service exposes an unsupported API version')
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(schema, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
    print(f"{len(schema['paths'])} paths exported to {args.output}")


if __name__ == '__main__':
    main()
