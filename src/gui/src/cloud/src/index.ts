let latestIP = null;

export default {
  async fetch(request) {
    const url = new URL(request.url);

    if (request.method === "POST") {
      try {
        const body = await request.text();
        latestIP = body;
        return new Response("IP received", { status: 200 });
      } catch (err) {
        return new Response("Error", { status: 500 });
      }
    }

    if (request.method === "GET") {
      return new Response(latestIP ?? "No IP set", { status: 200 });
    }

    return new Response("Method not allowed", { status: 405 });
  },
}
