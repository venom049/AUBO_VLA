"""JSON-RPC transport. Controller return codes are checked by the driver."""

import json
import math
import urllib.error
import urllib.request


class AuboRPCError(RuntimeError):
    def __init__(self, method, error):
        self.code = error.get("code")
        self.data = error.get("data")
        super().__init__(f"{method}: RPC error {self.code}: {error.get('message', error)}")


class AuboRPC:
    def __init__(self, url="http://127.0.0.1:9012/jsonrpc", timeout=5.0):
        self.url = url
        self.timeout = timeout
        self.req_id = 0

    def call(self, method, params=None, *, timeout=None):
        timeout = self.timeout if timeout is None else timeout
        if not math.isfinite(timeout) or timeout <= 0:
            raise ValueError("RPC timeout must be finite and positive")
        self.req_id += 1
        request_id = self.req_id
        payload = {
            "jsonrpc": "2.0", "method": method,
            "params": [] if params is None else params, "id": request_id,
        }
        request = urllib.request.Request(
            self.url, data=json.dumps(payload, allow_nan=False).encode("utf-8"),
            headers={"Content-Type": "application/json"}, method="POST",
        )
        try:
            with urllib.request.urlopen(request, timeout=timeout) as response:
                result = json.loads(response.read().decode("utf-8"))
        except (urllib.error.URLError, OSError) as exc:
            raise RuntimeError(f"{method}: cannot reach AUBO at {self.url}: {exc}") from exc
        except (ValueError, UnicodeError) as exc:
            raise RuntimeError(f"{method}: invalid JSON response") from exc
        if not isinstance(result, dict) or result.get("jsonrpc") != "2.0":
            raise RuntimeError(f"{method}: invalid JSON-RPC response")
        if result.get("id") != request_id or type(result.get("id")) is not int:
            raise RuntimeError(f"{method}: response ID does not match request {request_id}")
        if ("error" in result) == ("result" in result):
            raise RuntimeError(f"{method}: response must contain either result or error")
        if "error" in result:
            if not isinstance(result["error"], dict):
                raise RuntimeError(f"{method}: malformed RPC error")
            raise AuboRPCError(method, result["error"])
        return result["result"]
