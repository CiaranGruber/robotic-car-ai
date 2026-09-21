#!/usr/bin/env python3
"""Read-only release identity checks; never fetch, change refs or publish."""
import json
import os
from pathlib import Path
import re
import subprocess
import xml.etree.ElementTree as ET


ROOT = Path(__file__).resolve().parents[1]
VERSION = r'(0|[1-9][0-9]*)\.(0|[1-9][0-9]*)\.(0|[1-9][0-9]*)'


def require(condition, message):
    if not condition:
        raise ValueError(message)


def git(root, *args):
    return subprocess.check_output(
        ['git', '-C', str(root), *args], text=True).strip()


def promotion(root, version):
    parents = git(root, 'show', '-s', '--format=%P', 'HEAD').split()
    require(len(parents) == 2, 'release requires a two-parent promotion')
    require(git(root, 'show', '-s', '--format=%s', 'HEAD') ==
            'release: v' + version, 'wrong promotion title')
    git(root, 'merge-base', '--is-ancestor', parents[0], parents[1])
    git(root, 'merge-base', '--is-ancestor', parents[1], 'origin/dev')
    git(root, 'merge-base', '--is-ancestor', 'HEAD', 'origin/main')
    require(git(root, 'rev-parse', 'HEAD^{tree}') ==
            git(root, 'rev-parse', parents[1] + '^{tree}'),
            'promotion changes the candidate tree')
    return parents[1]


def check(root=ROOT, environment=None):
    environment = os.environ if environment is None else environment
    root = Path(root)
    metadata = ET.parse(root / 'package.xml').getroot()
    name, version = metadata.findtext('name'), metadata.findtext('version')
    require(re.fullmatch(r'[a-z][a-z0-9_]*', name or ''),
            'invalid package name')
    require(re.fullmatch(VERSION, version or ''), 'invalid package version')
    git(root, 'diff-tree', '-m', '--root', '--check', 'HEAD')
    require(not git(root, 'status', '--porcelain', '--untracked-files=all'),
            'source must be clean')
    commit = git(root, 'rev-parse', 'HEAD')
    expected = environment.get('CI_COMMIT_SHA')
    require(not expected or expected == commit, 'CI checkout differs')
    result = {'package': name, 'version': version, 'source_commit': commit,
              'source_tree': git(root, 'rev-parse', 'HEAD^{tree}')}

    tag = environment.get('CI_COMMIT_TAG', '')
    branch = environment.get('CI_COMMIT_BRANCH', '')
    if tag:
        match = re.fullmatch(r'v(' + VERSION + r')(-rc\.[1-9][0-9]*)?', tag)
        require(match is not None, 'unsupported release tag')
        require(match.group(1) == version, 'tag/version mismatch')
        ref = 'refs/tags/' + tag
        require(git(root, 'cat-file', '-t', ref) == 'tag',
                'release identity must be annotated')
        require(git(root, 'rev-parse', ref + '^{}') == commit,
                'tag points to another checkout')
        result['tag'] = tag
        result['tag_object'] = git(root, 'rev-parse', ref)
        if match.group(5):
            git(root, 'merge-base', '--is-ancestor', 'origin/main', 'HEAD')
            require(commit == git(root, 'rev-parse', 'origin/dev'),
                    'candidate tag differs from frozen dev')
            result['kind'] = 'candidate'
            result['candidate_commit'] = commit
        else:
            result['kind'] = 'release'
            result['candidate_commit'] = promotion(root, version)
    elif branch == 'main':
        result['candidate_commit'] = promotion(root, version)
    elif branch == 'dev':
        parents = git(root, 'show', '-s', '--format=%P', 'HEAD').split()
        require(len(parents) == 2, 'dev update requires a reviewed merge')
        git(root, 'merge-base', '--is-ancestor', parents[0], parents[1])

    source = environment.get('CI_MERGE_REQUEST_SOURCE_BRANCH_NAME')
    target = environment.get('CI_MERGE_REQUEST_TARGET_BRANCH_NAME')
    if source == 'main' and target == 'dev':
        git(root, 'merge-base', '--is-ancestor', 'origin/dev', 'HEAD')
        require(result['source_tree'] ==
                git(root, 'rev-parse', 'origin/dev^{tree}'),
                'synchronization changes content; reconcile separately')
    elif source == 'dev' and target == 'main':
        git(root, 'merge-base', '--is-ancestor', 'origin/main', 'HEAD')
        require(commit == git(root, 'rev-parse', 'origin/dev'),
                'release candidate differs from frozen dev')
    return result


if __name__ == '__main__':
    print(json.dumps(check(), indent=2))
