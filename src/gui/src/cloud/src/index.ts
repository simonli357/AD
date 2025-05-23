export default {
  async fetch(request: Request, env: { IP_STORE: KVNamespace }) {
    if (request.method === "POST") {
      try {
        const body = await request.text();
        await env.IP_STORE.put("latest_ip", body.trim());
        return new Response("IP received", { status: 200 });
      } catch (err) {
        return new Response("Error storing IP", { status: 500 });
      }
    }

    if (request.method === "GET") {
      const ip = await env.IP_STORE.get("latest_ip");
      return new Response(ip ?? "No IP set", { status: 200 });
    }

    return new Response("Method not allowed", { status: 405 });
  },
};
