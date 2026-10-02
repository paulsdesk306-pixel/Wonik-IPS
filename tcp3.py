#%%
# %%
import socket
from html.parser import HTMLParser


IP = "192.168.0.10"
PORT = 80


# ============================================================
# 1. HTTP GET으로 메인 페이지 HTML 읽기
# ============================================================

def get_html():

    with socket.socket(
        socket.AF_INET,
        socket.SOCK_STREAM
    ) as s:

        s.settimeout(3)

        print("TCP connecting...")

        s.connect(
            (IP, PORT)
        )

        print(
            "TCP CONNECTED:",
            s.getpeername()
        )

        request = (
            "GET / HTTP/1.0\r\n"
            f"Host: {IP}\r\n"
            "Connection: close\r\n"
            "\r\n"
        ).encode("ascii")

        s.sendall(request)

        response = b""

        while True:

            data = s.recv(4096)

            if not data:
                break

            response += data


    # Header / Body 분리
    header_raw, html_raw = response.split(
        b"\r\n\r\n",
        1
    )


    # 이 웹페이지는 GBK charset 사용
    html = html_raw.decode(
        "gbk",
        errors="replace"
    )

    return html


# ============================================================
# 2. HTML Form Parser
# ============================================================

class FormParser(HTMLParser):

    def __init__(self):

        super().__init__()

        self.forms = []

        self.current_form = None


    def handle_starttag(
        self,
        tag,
        attrs
    ):

        attrs = dict(attrs)


        # ----------------------------------------------------
        # FORM 시작
        # ----------------------------------------------------

        if tag.lower() == "form":

            self.current_form = {

                "method":
                    attrs.get(
                        "method",
                        "get"
                    ).upper(),

                "action":
                    attrs.get(
                        "action",
                        ""
                    ),

                "inputs":
                    []
            }


        # ----------------------------------------------------
        # FORM 내부 INPUT
        # ----------------------------------------------------

        elif (
            tag.lower() == "input"
            and
            self.current_form is not None
        ):

            self.current_form[
                "inputs"
            ].append({

                "type":
                    attrs.get(
                        "type",
                        ""
                    ),

                "name":
                    attrs.get(
                        "name",
                        ""
                    ),

                "value":
                    attrs.get(
                        "value",
                        ""
                    ),
            })


    def handle_endtag(
        self,
        tag
    ):

        if (
            tag.lower() == "form"
            and
            self.current_form is not None
        ):

            self.forms.append(
                self.current_form
            )

            self.current_form = None


# ============================================================
# 3. 실행
# ============================================================

html = get_html()

parser = FormParser()

parser.feed(
    html
)


# ============================================================
# 4. FORM 내용 전부 출력
# ============================================================

print()
print("=" * 80)
print("RAYCUS HTTP CGI FORMS")
print("=" * 80)


for number, form in enumerate(
    parser.forms,
    start=1
):

    print()
    print(
        f"[FORM {number}]"
    )

    print(
        "METHOD :",
        form["method"]
    )

    print(
        "ACTION :",
        form["action"]
    )


    for inp in form["inputs"]:

        print(
            "    ",
            f"type={inp['type']:<10}",
            f"name={inp['name']:<20}",
            f"value={inp['value']}"
        )
# %%
