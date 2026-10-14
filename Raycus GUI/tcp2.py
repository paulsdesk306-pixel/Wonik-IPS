#%%
import socket
import re

IP = "192.168.0.10"
PORT = 80

# ------------------------------------------------------------
# 1. HTTP GET으로 메인 페이지 읽기
# ------------------------------------------------------------

with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    s.settimeout(3)
    s.connect((IP, PORT))

    request = (
        "GET / HTTP/1.0\r\n"
        f"Host: {IP}\r\n"
        "\r\n"
    ).encode("ascii")

    s.sendall(request)

    response = b""

    while True:
        data = s.recv(4096)

        if not data:
            break

        response += data


# ------------------------------------------------------------
# 2. GBK로 HTML decode
#
# 페이지 meta charset이 GBK이므로
# utf-8보다 GBK로 읽는 게 맞음.
# ------------------------------------------------------------

text = response.decode(
    "gbk",
    errors="replace"
)


# HTTP header와 HTML 분리
if "\r\n\r\n" in text:
    header, html = text.split("\r\n\r\n", 1)
else:
    header = ""
    html = text


print("=" * 70)
print("HTTP HEADER")
print("=" * 70)
print(header)


# ------------------------------------------------------------
# 3. form action 찾기
# ------------------------------------------------------------

print()
print("=" * 70)
print("FORM ACTIONS")
print("=" * 70)

actions = re.findall(
    r'<form[^>]*action=["\']?([^"\'>\s]+)',
    html,
    flags=re.IGNORECASE
)

for action in actions:
    print(action)


# ------------------------------------------------------------
# 4. href 찾기
# ------------------------------------------------------------

print()
print("=" * 70)
print("LINKS")
print("=" * 70)

links = re.findall(
    r'href=["\']([^"\']+)["\']',
    html,
    flags=re.IGNORECASE
)

for link in links:
    print(link)


# ------------------------------------------------------------
# 5. onclick 찾기
# ------------------------------------------------------------

print()
print("=" * 70)
print("ONCLICK")
print("=" * 70)

onclicks = re.findall(
    r'onclick=["\']([^"\']+)["\']',
    html,
    flags=re.IGNORECASE
)

for item in onclicks:
    print(item)


# ------------------------------------------------------------
# 6. input name/value 찾기
# ------------------------------------------------------------

print()
print("=" * 70)
print("INPUT FIELDS")
print("=" * 70)

inputs = re.findall(
    r'<input[^>]*>',
    html,
    flags=re.IGNORECASE
)

for item in inputs:
    print(item)


# ------------------------------------------------------------
# 7. script source / JavaScript 함수 흔적
# ------------------------------------------------------------

print()
print("=" * 70)
print("SCRIPT / CGI KEYWORDS")
print("=" * 70)

for line in html.splitlines():

    lower = line.lower()

    if any(
        keyword in lower
        for keyword in [
            "cgi",
            "ajax",
            "xmlhttprequest",
            "fetch(",
            "form",
            "onclick",
            "submit",
        ]
    ):
        print(line.strip())
# %%
