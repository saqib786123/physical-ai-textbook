FROM node:20

WORKDIR /usr/src/app

COPY package*.json ./

RUN npm install

COPY . .

# Koyeb usually prefers port 8000 or the PORT env var
EXPOSE 8000

ENV PORT=8000

CMD [ "node", "server.js" ]
