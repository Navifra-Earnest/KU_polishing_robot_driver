#!/usr/bin/env python3
# =============================================================================
#  sync_param.py <template.yaml> <user.yaml>
#
#  템플릿(template)에는 있고 사용자 파일(user)에는 없는 키를, 사용자 파일의
#  기존 값·주석·서식을 그대로 둔 채 "추가만" 한다(additive merge).
#    - 기존 사용자 값은 절대 바꾸지 않는다(현장 편집 보존).
#    - 새로 생긴 최상위 노드는 통째로 파일 끝에 추가.
#    - 기존 노드에 없는 하위 키는 그 노드 블록 끝에 템플릿 원문(주석 포함)으로 삽입.
#  user 파일이 없으면 template 를 그대로 복사한다.
#
#  주석 보존을 위해 PyYAML 은 "구조 파악"에만 쓰고, 실제 삽입은 원문 라인 단위로 한다.
#  (ruamel.yaml 불필요 — 로봇에 PyYAML 만 있어도 동작)
# =============================================================================
import os
import sys

try:
    import yaml
except ImportError:
    sys.stderr.write("sync_param: PyYAML 필요 (python3-yaml)\n")
    sys.exit(1)


def top_key(line):
    """최상위 노드 헤더면 이름 반환 (예: 'lift_driver:'), 아니면 None."""
    if line and line[0] not in (' ', '\t', '#') and line.rstrip().endswith(':'):
        return line.split(':', 1)[0].strip()
    return None


def sub_key(line):
    """2칸 들여쓰기 하위 키면 이름 반환, 아니면 None."""
    if line.startswith('  ') and line[2:3] not in (' ', '#', '') :
        stripped = line.strip()
        if ':' in stripped and not stripped.startswith('#'):
            return stripped.split(':', 1)[0].strip()
    return None


def parse_blocks(lines):
    """최상위 노드별 (헤더 idx, 끝 idx exclusive) 와 하위키별 원문 라인 범위를 반환."""
    nodes = {}          # node -> {'start':i,'end':j,'keys':{key:(a,b)}}
    cur = None
    for i, line in enumerate(lines):
        tk = top_key(line)
        if tk is not None:
            cur = tk
            nodes[cur] = {'start': i, 'end': len(lines), 'keys': {}}
            continue
        if cur is not None and top_key(lines[i]) is None:
            # 이전 노드 end 를 갱신(다음 최상위 노드 만나기 전까지)
            nodes[cur]['end'] = i + 1
        sk = sub_key(line)
        if cur is not None and sk is not None:
            # 이 하위키의 원문 범위 = 이 줄 + 뒤따르는 더 깊은 들여쓰기 연속줄(연장 주석 등)
            a = i
            b = i + 1
            while b < len(lines):
                nxt = lines[b]
                if nxt.strip() == '' or top_key(nxt) is not None or sub_key(nxt) is not None:
                    break
                if nxt.startswith('   '):   # 키(2칸)보다 깊은 들여쓰기 → 연장줄
                    b += 1
                else:
                    break
            nodes[cur]['keys'][sk] = (a, b)
    # content_end = 마지막 실제 하위키 블록의 끝(뒤따르는 빈줄/섹션주석 앞). 삽입 지점.
    for n in nodes.values():
        n['content_end'] = max([b for (_, b) in n['keys'].values()], default=n['start'] + 1)
    return nodes


def main():
    if len(sys.argv) != 3:
        sys.stderr.write("usage: sync_param.py <template.yaml> <user.yaml>\n")
        sys.exit(2)
    tmpl_path, user_path = sys.argv[1], sys.argv[2]

    with open(tmpl_path, 'r') as f:
        tmpl_text = f.read()
    tmpl_lines = tmpl_text.splitlines()

    # user 없으면 template 복사
    if not os.path.exists(user_path):
        with open(user_path, 'w') as f:
            f.write(tmpl_text if tmpl_text.endswith('\n') else tmpl_text + '\n')
        print("created (from template)")
        return

    with open(user_path, 'r') as f:
        user_lines = f.read().splitlines()

    tmpl_struct = yaml.safe_load(tmpl_text) or {}
    user_struct = yaml.safe_load('\n'.join(user_lines)) or {}

    tmpl_blocks = parse_blocks(tmpl_lines)
    user_blocks = parse_blocks(user_lines)

    added = []
    # 결과를 인덱스 기반으로 조립하기 위해, 삽입은 뒤에서부터(인덱스 밀림 방지) 적용
    inserts = []  # (position, [lines])

    for node, keys in tmpl_struct.items():
        if node not in user_struct:
            # 신규 노드 전체를 파일 끝에 추가
            tb = tmpl_blocks[node]
            block = lines_slice(tmpl_lines, tb['start'], tb['end'])
            inserts.append((len(user_lines), [''] + block))
            added.append(node + " (신규 노드)")
            continue
        if not isinstance(keys, dict):
            continue
        ub = user_blocks[node]
        for key in keys:
            if key in user_struct.get(node, {}):
                continue
            # 템플릿에서 이 키의 원문 라인 추출
            if key not in tmpl_blocks[node]['keys']:
                continue
            a, b = tmpl_blocks[node]['keys'][key]
            raw = tmpl_lines[a:b]
            # 사용자 파일의 해당 노드 마지막 키 뒤(섹션 주석/빈줄 앞)에 삽입
            inserts.append((ub['content_end'], raw))
            added.append("%s.%s" % (node, key))

    if not added:
        print("up-to-date (no keys added)")
        return

    # 뒤에서부터 삽입
    result = list(user_lines)
    for pos, block in sorted(inserts, key=lambda x: -x[0]):
        result[pos:pos] = block

    with open(user_path, 'w') as f:
        f.write('\n'.join(result) + '\n')
    print("added: " + ", ".join(added))


def lines_slice(lines, a, b):
    return lines[a:b]


if __name__ == '__main__':
    main()
