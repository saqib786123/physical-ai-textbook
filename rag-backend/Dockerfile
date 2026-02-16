FROM node:20

WORKDIR /usr/src/app

COPY package*.json ./

RUN npm install

COPY . .

# Hugging Face Spaces usually runs on port 7860
EXPOSE 7860

ENV PORT=7860

CMD [ "node", "server.js" ]
